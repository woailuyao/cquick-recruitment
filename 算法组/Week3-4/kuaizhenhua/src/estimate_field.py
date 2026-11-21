"""Approximate plane from a manually identified white rectangle, unknown scale.

Camera calibration is from this camera's chessboard recording. Rectangle width
is defined as ONE arbitrary unit, not one metre. This is not a surveyed map.
"""
from pathlib import Path
import json,math
from datetime import datetime
import cv2,numpy as np
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--image',type=Path,required=True,help='clip005 frame0, original 1280x720')
parser.add_argument('--intrinsics',type=Path,default=Path('calibration/camera_intrinsics.yaml'))
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();OUT=args.output;OUT.mkdir(parents=True,exist_ok=True)
import yaml
a=yaml.safe_load(args.intrinsics.read_text(encoding='utf-8'))
K=np.array(a['camera_matrix']);D=np.array(a['dist_coeffs'])
image=cv2.imdecode(np.fromfile(args.image,np.uint8),1)
# Line centre intersections, inspected in original 1280x720 image. Order: far
# left, far right, near right, near left; they bound one white-line rectangle.
raw=np.array([[250,151],[1163,174],[1233,219],[171,197]],np.float64)
und=cv2.undistortPoints(raw[:,None],K,D,P=K).reshape(-1,2)
square=np.array([[0,0],[1,0],[1,1],[0,1]],np.float32)
H0=cv2.getPerspectiveTransform(square,und.astype(np.float32))
g=np.linalg.inv(K)@H0
ratio=np.linalg.norm(g[:,1])/np.linalg.norm(g[:,0])
orthogonality=float(np.dot(g[:,0],g[:,1])/np.linalg.norm(g[:,0])/np.linalg.norm(g[:,1]))
# World +X toward goal (-rectangle depth), +Y to observer's left (-width).
world=np.array([[0,0,0],[0,-1,0],[-ratio,-1,0],[-ratio,0,0]],np.float64)
ok,rvec,tvec=cv2.solvePnP(world,raw,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
assert ok
R,_=cv2.Rodrigues(rvec);C=-R.T@tvec.reshape(3)
H=K@np.column_stack((R[:,0],R[:,1],tvec.reshape(3)))
Hi=np.linalg.inv(H)
def to_ground(pixels):
    q=cv2.undistortPoints(np.asarray(pixels,np.float64).reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    h=np.c_[q,np.ones(len(q))]@Hi.T
    return h[:,:2]/h[:,2:]
reproj,_=cv2.projectPoints(world,rvec,tvec,K,D)
err=np.linalg.norm(reproj.reshape(-1,2)-raw,axis=1)
# Fit the visible centre-circle arc after plane rectification. Restrict to arc
# band; the bottom centre line is excluded. The fit is approximate, not survey.
hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
mask=(hsv[:,:,1]<65)&(hsv[:,:,2]>180)
yy,xx=np.nonzero(mask)
keep=(yy>430)&(yy<650)&(xx>80)&(xx<1200)
pixels=np.c_[xx[keep],yy[keep]][::8].astype(float)
ground=to_ground(pixels)
M=np.c_[2*ground[:,0],2*ground[:,1],np.ones(len(ground))]
fit=np.linalg.lstsq(M,(ground**2).sum(1),rcond=None)[0]
center=fit[:2];radius=np.sqrt(max(0,fit[2]+sum(center**2)))
circle_err=np.linalg.norm(ground-center,axis=1)-radius
# Remove points > robust tolerance then refit.
inliers=np.abs(circle_err-np.median(circle_err))<max(.015,3*np.median(np.abs(circle_err-np.median(circle_err))))
fit=np.linalg.lstsq(M[inliers],(ground[inliers]**2).sum(1),rcond=None)[0]
center=fit[:2];radius=np.sqrt(max(0,fit[2]+sum(center**2)))
marker_pixels={'LCross1':[292,329],'TCross1':[634,330],'XCross1':[964,345]}
coords=to_ground(list(marker_pixels.values()))-center
markers={name:[round(float(v),6) for v in xy] for name,xy in zip(marker_pixels,coords)}
Cmap=C-np.r_[center,0]
result={
 'status':'video_estimate_relative_only','metric_scale_verified':False,
 'unit':'reference_white_rectangle_width','metres_per_unit':None,
 'origin':'approximate centre of visible centre-circle fitted after rectification',
 'axis_x':'toward goal, parallel rectangle depth','axis_y':'left when facing +X','axis_z':'up',
 'source_clip':'clip_005_20261116T160612_868254_30dcf8b6.avi','source_frame_index_0based':0,
 'created_at':datetime.now().astimezone().isoformat(),
 'white_rectangle_pixels':raw.tolist(),'rectangle_depth_over_width':float(ratio),
 'rectangle_orthogonality_cosine_before_projection':orthogonality,
 'rectangle_pnp_reprojection_rmse_px':float(np.sqrt(np.mean(err**2))),
 'camera_matrix':K.tolist(),'dist_coeffs':D.tolist(),
 'rotation_camera_to_reference':R.T.tolist(),'camera_center_reference':C.tolist(),
 'circle_center_reference_xy':center.tolist(),'circle_radius_units':float(radius),
 'camera_center_map_units':Cmap.tolist(),
 'homography_reference_ground_to_undistorted_pixel':H.tolist(),
 'marker_pixels':marker_pixels,'markers':markers,
 'limitations':['Rectangle corner pixels selected visually; physical rectangle and circle geometry only approximated.',
 'Unknown absolute length scale; all translations and radius expressed in rectangle-width units.',
 'Three tape points are nearly collinear; pose ambiguity and geometry must be rejected when unstable.',
 'Camera tilt/height assumed unchanged when using the fixed ray-plane model.',
 'Reprojection residual is internal fit consistency, not metric position accuracy.',
 'PenaltyPoint has no visible labelled target and is not inserted as a detected marker.']}
(OUT/'field_geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'field.yaml').write_text('# VIDEO ESTIMATE; unit = white rectangle width; NOT metres\n'+''.join(f'{k}: {v}\n' for k,v in markers.items()),encoding='utf-8')
vis=image.copy()
for i,q in enumerate(raw.astype(int)):
    cv2.circle(vis,tuple(q),6,(0,0,255),-1);cv2.putText(vis,'R'+str(i),tuple(q+[-10,-12]),0,.6,(0,0,255),2)
for name,p in marker_pixels.items():
    cv2.drawMarker(vis,tuple(p),(255,0,255),cv2.MARKER_CROSS,18,2)
    cv2.putText(vis,name,tuple(np.array(p)+[-30,-18]),0,.6,(255,0,255),2)
cv2.imencode('.png',vis)[1].tofile(str(OUT/'reference_annotations.png'))
print(json.dumps({k:result[k] for k in ['markers','rectangle_depth_over_width','rectangle_orthogonality_cosine_before_projection','rectangle_pnp_reprojection_rmse_px','circle_radius_units','camera_center_map_units']},indent=2))
