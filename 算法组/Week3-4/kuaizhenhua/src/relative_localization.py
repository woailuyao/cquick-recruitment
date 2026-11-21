"""Video-derived ground geometry. All translations use rectangle-width units.

This estimates the camera optical centre, NOT a measured robot pose. Height
and tilt must stay fixed. Unknown metric scale always keeps valid=False.
"""
from pathlib import Path
import math
import cv2
import numpy as np
import yaml
from field_map_matching import load_field_map, match_ground_points


def skeleton(mask):
    """Zhang-Suen thinning; one connected centre line per tape arm."""
    a=np.pad((mask>0).astype(np.uint8),1)
    for _ in range(150):
        changed=False
        for phase in (0,1):
            p=[a[:-2,1:-1],a[:-2,2:],a[1:-1,2:],a[2:,2:],a[2:,1:-1],a[2:,:-2],a[1:-1,:-2],a[:-2,:-2]]
            neighbours=sum(p)
            changes=sum((p[i]==0)&(p[(i+1)%8]==1) for i in range(8))
            condition=(p[0]*p[2]*p[4]==0)&(p[2]*p[4]*p[6]==0) if phase==0 else (p[0]*p[2]*p[6]==0)&(p[0]*p[4]*p[6]==0)
            remove=(a[1:-1,1:-1]==1)&(neighbours>=2)&(neighbours<=6)&(changes==1)&condition
            if remove.any():a[1:-1,1:-1][remove]=0;changed=True
        if not changed:break
    return a[1:-1,1:-1]*255


def junction(image, bbox, class_name='LCross'):
    """Intersect two tape centre lines inside a detector ROI; no box-centre fallback."""
    h,w=image.shape[:2]
    x1,y1,x2,y2=np.rint(bbox).astype(int)
    x1,y1=max(0,x1),max(0,y1);x2,y2=min(w,x2),min(h,y2)
    if x2-x1<6 or y2-y1<6:return None
    roi=image[y1:y2,x1:x2]
    # Small marks are upscaled for skeleton/line extraction, not for inference.
    scale=min(3.,160/max(roi.shape[:2]))
    hsv=cv2.cvtColor(cv2.resize(roi,None,fx=scale,fy=scale),cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(12,40,140),(42,255,255)) | cv2.inRange(hsv,(0,0,220),(179,80,255))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    n,labels,stats,centres=cv2.connectedComponentsWithStats(mask)
    if n<2:return None
    areas=stats[1:,cv2.CC_STAT_AREA]; chosen=1+int(areas.argmax())
    if areas.max()<30:return None
    mask=np.uint8(labels==chosen)*255
    thin=skeleton(mask)
    if class_name in ('TCross','XCross'):
        binary=(thin>0).astype(np.uint8)
        neighbours=cv2.filter2D(binary,cv2.CV_16S,np.ones((3,3),np.int16))-binary
        branches=np.uint8((binary>0)&(neighbours>=3))
        nb,bl,bs,bc=cv2.connectedComponentsWithStats(branches)
        options=[]
        for j in range(1,nb):
            center=bc[j];remaining=thin.copy()
            cv2.circle(remaining,tuple(np.rint(center).astype(int)),4,0,-1)
            count,_,sizes,_=cv2.connectedComponentsWithStats(remaining)
            arms=sum(sizes[k,cv2.CC_STAT_AREA]>=6 for k in range(1,count))
            if arms >= (3 if class_name=='TCross' else 4):
                options.append((np.linalg.norm(center-centres[chosen]),center))
        if options:
            point=min(options,key=lambda v:v[0])[1]
            return [float(x1+point[0]/scale),float(y1+point[1]/scale)]
        # Foreshortened short arms can disappear from the skeleton graph;
        # try the same two-centre-line fit, still requiring both line supports.
    lines=cv2.HoughLinesP(thin,1,np.pi/180,threshold=8,minLineLength=8,maxLineGap=5)
    if lines is None:return None
    lines=sorted(lines[:,0].astype(float),key=lambda l:-np.linalg.norm(l[2:]-l[:2]))[:30]
    best=None
    sy,sx=np.nonzero(thin);cloud=np.column_stack((sx,sy))
    for i,a in enumerate(lines):
        p=a[:2];d=a[2:]-p;ln=np.linalg.norm(d)
        for b in lines[i+1:]:
            q=b[:2];e=b[2:]-q;le=np.linalg.norm(e)
            sine=abs(np.linalg.det(np.column_stack((d,e))))/(ln*le)
            if sine<.15:continue
            t=np.linalg.solve(np.column_stack((d,-e)),q-p)
            cross=p+t[0]*d
            xx,yy=np.rint(cross).astype(int)
            if not(0<=xx<mask.shape[1] and 0<=yy<mask.shape[0]) or not mask[yy,xx]:continue
            # Permit short extrapolation to the L/T joint but reject remote lines.
            if np.any(t<-.35) or np.any(t>1.35):continue
            da=np.abs((cloud-p)[:,0]*d[1]-(cloud-p)[:,1]*d[0])/ln
            db=np.abs((cloud-q)[:,0]*e[1]-(cloud-q)[:,1]*e[0])/le
            support_a=np.sum((da<2)&(da<db));support_b=np.sum((db<2)&(db<=da))
            if min(support_a,support_b)<5:continue
            score=np.sum(np.minimum(da,db)<2)+.2*min(support_a,support_b)
            if best is None or score>best[0]:best=(score,cross)
    if best is None:return None
    return [float(x1+best[1][0]/scale),float(y1+best[1][1]/scale)]


class RelativeLocalizer:
    def __init__(self,geometry_path,field_path):
        self.geometry=yaml.safe_load(Path(geometry_path).read_text(encoding='utf-8-sig'))
        g=self.geometry
        self.K=np.asarray(g['camera_matrix'],float);self.D=np.asarray(g['dist_coeffs'],float)
        self.R=np.asarray(g['rotation_camera_to_reference'],float)
        self.C=np.asarray(g['camera_center_reference'],float)
        if not np.isfinite(self.R).all() or not np.allclose(self.R.T@self.R,np.eye(3),atol=1e-5) or self.C[2]<=0:
            raise ValueError('Invalid ground-plane geometry')
        self.landmarks,self.map_info=load_field_map(Path(field_path))
        self.map_info['unit']=g['unit']  # map parser itself makes no physical unit measurement
        self.base_yaw=math.degrees(math.atan2(self.R[1,2],self.R[0,2]))

    def ground_point(self,pixel):
        ray=np.r_[cv2.undistortPoints(np.asarray(pixel,float).reshape(1,1,2),self.K,self.D).reshape(2),1.]
        direction=self.R@ray
        if direction[2]>=-1e-4:return None
        distance=-self.C[2]/direction[2]
        p=distance*direction
        if distance<=0 or np.linalg.norm(p[:2])>5:return None
        return p[:2]

    def estimate_points(self,points):
        observations=[]
        for item in points:
            p=self.ground_point(item['pixel'])
            if p is not None:observations.append({**item,'x':float(p[0]),'y':float(p[1])})
        result=match_ground_points(observations,self.landmarks,min_markers=3,
            gate_m=.055,max_residual_m=.025,min_baseline_m=.08,
            ambiguity_residual_gap_m=.005,distinct_pose_distance_m=.1)
        valid=result['valid']
        yaw=(result['yaw_carrier_map_deg']+self.base_yaw+180)%360-180 if valid else None
        return {'valid':False,'metric_valid':False,'relative_valid':valid,
            'status':'relative_only_scale_unverified' if valid else result['status'],
            'x_camera_map':None,'y_camera_map':None,'yaw_camera_map':None,
            'x_camera_map_units':result['x_carrier_map'] if valid else None,
            'y_camera_map_units':result['y_carrier_map'] if valid else None,
            'yaw_camera_map_deg':yaw,'marker_count':result['marker_count'],
            'residual_units':result['residual'],'unit':self.geometry['unit'],
            'matched_ids':result['matched_ids'],'observations':observations,
            'assumption':'video-estimated plane; unchanged camera height and tilt',
            'field_sha256':self.map_info['sha256']}

    def estimate(self,detections,image):
        if image.shape[:2]!=(720,1280):
            result=self.estimate_points([]);result['status']='calibration_resolution_mismatch';return result
        points=[]
        for det in sorted(detections,key=lambda d:-d['confidence'])[:14]:
            if det['class_name'] not in ('LCross','TCross','XCross'):continue
            pixel=junction(image,det['bbox'],det['class_name'])
            if pixel is not None:points.append({'class_name':det['class_name'],'pixel':pixel,'confidence':det['confidence']})
        return self.estimate_points(points)
