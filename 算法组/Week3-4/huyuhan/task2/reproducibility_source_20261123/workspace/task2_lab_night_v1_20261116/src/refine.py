"""White-stripe geometric refinement prototype. Failure has no bbox-center fallback."""
import numpy as np,cv2

def refine(image,box,cls):
 x1,y1,x2,y2=map(int,box);h,w=image.shape[:2];x1,y1=max(0,x1),max(0,y1);x2,y2=min(w,x2),min(h,y2);roi=image[y1:y2,x1:x2]
 if min(roi.shape[:2],default=0)<12:return None
 hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,np.array([0,0,150]),np.array([179,100,255]));mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
 if cls==3:
  n,labels,stats,cent=cv2.connectedComponentsWithStats(mask);ids=[i for i in range(1,n) if 8<stats[i,4]<mask.size*.45 and max(stats[i,2:4])/max(1,min(stats[i,2:4]))<2.5]
  return (cent[ids[0]]+[x1,y1]).tolist() if len(ids)==1 else None
 skeleton=np.zeros_like(mask);work=mask.copy();el=cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
 for _ in range(max(mask.shape)):
  er=cv2.erode(work,el);skeleton|=cv2.subtract(work,cv2.dilate(er,el));work=er
  if cv2.countNonZero(work)==0:break
 seg=cv2.HoughLinesP(skeleton,1,np.pi/180,threshold=max(7,min(mask.shape)//6),minLineLength=max(10,min(mask.shape)//4),maxLineGap=7)
 if seg is None:return None
 seg=np.asarray(seg).reshape(-1,4).astype(float);seg=seg[np.argsort(-np.linalg.norm(seg[:,2:]-seg[:,:2],axis=1))[:12]];yy,xx=np.nonzero(mask);pixels=np.column_stack([xx,yy]).astype(float)
 if len(pixels)>1200:pixels=pixels[np.linspace(0,len(pixels)-1,1200,dtype=int)]
 center=np.array([mask.shape[1]/2,mask.shape[0]/2]);candidates=[]
 for ai,a in enumerate(seg):
  v=a[2:]-a[:2];v/=max(np.linalg.norm(v),1e-9)
  for b in seg[ai+1:]:
   v2=b[2:]-b[:2];v2/=max(np.linalg.norm(v2),1e-9);cross=abs(np.linalg.det(np.column_stack([v,v2])))
   if cross<.28:continue
   normals=np.array([[-v[1],v[0]],[-v2[1],v2[0]]]);offsets=np.array([a[:2]@normals[0],b[:2]@normals[1]]);good=True
   for iteration in range(5):
    try:point=np.linalg.solve(normals,offsets)
    except np.linalg.LinAlgError:good=False;break
    dd=np.abs(pixels@normals.T-offsets);assignment=np.argmin(dd,axis=1);newn=[];newc=[]
    for li in range(2):
     keep=(assignment==li)&(dd[:,li]<max(5,min(mask.shape)*.09))&(np.linalg.norm(pixels-point,axis=1)>min(mask.shape)*.13);pts=pixels[keep]
     if len(pts)<12:good=False;break
     _,_,vt=np.linalg.svd(pts-pts.mean(0),full_matrices=False);normal=vt[-1]
     if abs(normal@normals[li])<.90:good=False;break
     newn.append(normal);newc.append(float(np.median(pts@normal)))
    if not good:break
    normals,offsets=np.array(newn),np.array(newc)
   if not good:continue
   try:point=np.linalg.solve(normals,offsets)
   except np.linalg.LinAlgError:continue
   if np.any(point<2) or point[0]>mask.shape[1]-3 or point[1]>mask.shape[0]-3 or np.linalg.norm((point-center)/np.maximum(center,1))>.85:continue
   arms=0
   for direction in [v,-v,v2,-v2]:
    r=np.arange(5,max(9,min(mask.shape)*.30));p=point+r[:,None]*direction;xx,yy=np.rint(p[:,0]).astype(int),np.rint(p[:,1]).astype(int);inside=(xx>=0)&(xx<mask.shape[1])&(yy>=0)&(yy<mask.shape[0])
    if inside.sum()>=4 and (mask[yy[inside],xx[inside]]>0).mean()>.55:arms+=1
   if arms<{0:2,1:3,2:4}.get(cls,4):continue
   candidates.append((np.linalg.norm((point-center)/np.maximum(center,1))-.05*cross,point))
 if not candidates:return None
 candidates.sort(key=lambda x:x[0]);score,point=candidates[0]
 if any(s<score+.08 and np.linalg.norm(p-point)>10 for s,p in candidates[1:]):return None
 return (point+[x1,y1]).tolist()
