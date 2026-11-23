"""Capture/check camera, calibrate intrinsics, and calibrate rigid ground extrinsics."""
import argparse,csv,json,time
from datetime import datetime
from pathlib import Path
import cv2,numpy as np,yaml
from geometry import transform,pixel_ground

def camera(source):
 if str(source).startswith('bridge:'):
  from bridge_camera import BridgeCamera
  return BridgeCamera(str(source)[7:])
 cap=cv2.VideoCapture(int(source) if str(source).isdigit() else str(source))
 if not cap.isOpened():raise RuntimeError('无法打开相机；先检查WSL USB转发、设备占用及/dev/video*。')
 cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);cap.set(cv2.CAP_PROP_FPS,30);return cap
def read(cap):
 ok,im=cap.read()
 if not ok:raise RuntimeError('相机取帧失败/连接中断。')
 if im.shape[:2]!=(720,1280):raise RuntimeError('实际分辨率不是1280x720：'+str(im.shape))
 return im
def save_yaml(path,data):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 if path.exists():path=path.with_name(path.stem+'_'+str(time.time_ns())+path.suffix)
 path.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True));print('保存：',path);return path
def check(a):
 a.out.mkdir(parents=True,exist_ok=False);cap=camera(a.camera);start=time.perf_counter();frames=0;im=None
 try:
  while time.perf_counter()-start<a.seconds:im=read(cap);frames+=1
  elapsed=time.perf_counter()-start;cv2.imwrite(str(a.out/'reference.jpg'),im);r=dict(timestamp=datetime.now().astimezone().isoformat(),source=a.camera,width=1280,height=720,frames=frames,elapsed_s=elapsed,actual_capture_only_fps=frames/elapsed,nominal_fps=cap.get(cv2.CAP_PROP_FPS));(a.out/'camera_report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2));print(r)
 finally:cap.release()
def intrinsic(a):
 a.folder.mkdir(parents=True,exist_ok=True);size=(a.cols,a.rows)
 if a.square_mm<=0:raise ValueError('使用打印后实测方格边长，必须大于0。')
 if not a.solve_only:
  cap=camera(a.camera);last=None
  try:
   while True:
    im=read(cap);found,pts=cv2.findChessboardCornersSB(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),size,flags=cv2.CALIB_CB_NORMALIZE_IMAGE);vis=im.copy()
    if found:cv2.drawChessboardCorners(vis,size,pts,found)
    cv2.putText(vis,'SPACE save | move board: center/corners/tilt/distance | Q solve',(12,32),0,.65,(0,255,255),2);cv2.imshow('intrinsic calibration',vis);key=cv2.waitKey(1)&255
    if key==32:
     if not found:print('未检测到完整内角点，未保存。');continue
     normalized=pts.reshape(-1,2)/[1280,720]
     if last is not None and np.linalg.norm(normalized-last,axis=1).mean()<.025:print('请变换位置或倾角，不重复保存近似相同姿态。');continue
     cv2.imwrite(str(a.folder/f'board_{time.time_ns()}.png'),im);last=normalized;print('已保存',len(list(a.folder.glob('board_*.png'))),'张')
    if key==ord('q'):break
  finally:cap.release();cv2.destroyAllWindows()
 obj=np.zeros((a.cols*a.rows,3),np.float32);obj[:,:2]=np.mgrid[0:a.cols,0:a.rows].T.reshape(-1,2)*a.square_mm/1000;objects=[];points=[];files=[]
 for p in sorted(a.folder.glob('board_*.png')):
  im=cv2.imread(str(p))
  if im is None or im.shape[:2]!=(720,1280):continue
  found,pts=cv2.findChessboardCornersSB(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),size,flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
  if found:objects.append(obj.copy());points.append(pts);files.append(str(p))
 if len(files)<15:raise ValueError(f'仅{len(files)}张有效标定图，至少15张，建议20–25张多姿态图。')
 rms,K,dist,rv,tv=cv2.calibrateCamera(objects,points,(1280,720),None,None);errors=[]
 for pts,r,t in zip(points,rv,tv):
  projected,_=cv2.projectPoints(obj,r,t,K,dist);errors.append(float(np.sqrt(np.mean(np.sum((pts.reshape(-1,2)-projected.reshape(-1,2))**2,axis=1)))))
 good=bool(np.isfinite(K).all() and np.isfinite(dist).all() and K[0,0]>0 and K[1,1]>0 and rms<1.)
 save_yaml(a.out,dict(schema='cquick_intrinsics_v1',verified=good,width=1280,height=720,camera_matrix=K.tolist(),dist_coeffs=dist.ravel().tolist(),rms_pixels=float(rms),per_view_rms_pixels=errors,views=len(files),files=files,measured_square_m=a.square_mm/1000,inner_corners=[a.cols,a.rows]));print('RMS pixels:',rms,'verified:',good)
 if not good:raise ValueError('标定误差过高，保留原图；检查反光/模糊及图像覆盖后重拍。')
def ground(a):
 intr=yaml.safe_load(a.intrinsics.read_text());im=cv2.imread(str(a.image));records=list(csv.DictReader(a.points_csv.open(encoding='utf-8-sig')))
 if not intr.get('verified') or im is None or [im.shape[1],im.shape[0]]!=[intr['width'],intr['height']]:raise ValueError('需要有效内参及同分辨率参考图。')
 fi=[i for i,r in enumerate(records) if r['role']=='fit'];ci=[i for i,r in enumerate(records) if r['role']=='check']
 if len(fi)<6 or len(ci)<2:raise ValueError('至少6个fit点和2个独立check点，全部先实测坐标。')
 world=np.array([[float(r['x_forward_m']),float(r['y_left_m']),0.] for r in records])
 if not np.isfinite(world).all() or np.linalg.svd(world[fi,:2]-world[fi,:2].mean(0),compute_uv=False)[-1]<.2:raise ValueError('标定点应非共线并覆盖足够地面面积。')
 pixels=[]
 def clicked(event,x,y,flags,param):
  if event==cv2.EVENT_LBUTTONDOWN and len(pixels)<len(records):pixels.append([x,y])
 cv2.namedWindow('ground measured points',cv2.WINDOW_NORMAL);cv2.resizeWindow('ground measured points',1280,720);cv2.setMouseCallback('ground measured points',clicked)
 while True:
  vis=im.copy()
  for i,p in enumerate(pixels):cv2.circle(vis,p,5,(0,0,255),-1);cv2.putText(vis,records[i]['id'],p,0,.5,(0,0,255),1)
  row=records[len(pixels)] if len(pixels)<len(records) else None;label=f"Click {row['id']} x={row['x_forward_m']} y={row['y_left_m']} m" if row else 'ENTER solve | U undo | Q cancel';cv2.putText(vis,label,(12,30),0,.7,(0,255,255),2);cv2.imshow('ground measured points',vis);key=cv2.waitKey(20)&255
  if key==ord('u') and pixels:pixels.pop()
  if key==ord('q'):cv2.destroyAllWindows();raise ValueError('取消：没有写入外参。')
  if key in [10,13] and row is None:break
 cv2.destroyAllWindows();uv=np.asarray(pixels,float);K=np.asarray(intr['camera_matrix']);dist=np.asarray(intr['dist_coeffs']);ok,rvs,tvs,*_=cv2.solvePnPGeneric(world[fi],uv[fi],K,dist,flags=cv2.SOLVEPNP_IPPE);candidates=[]
 for rv,tv in zip(rvs,tvs):
  Rcw,_=cv2.Rodrigues(rv);R=Rcw.T;t=(-R@tv).ravel()
  if t[2]<=0 or abs(t[2]-a.measured_height)>.1 or np.any((world[fi]@Rcw.T+tv.ravel())[:,2]<=0):continue
  pred,_=cv2.projectPoints(world[fi],rv,tv,K,dist);rms=float(np.sqrt(np.mean(np.sum((uv[fi]-pred.reshape(-1,2))**2,axis=1))));xy=pixel_ground(uv[ci],K,dist,R,t)
  if any(x is None for x in xy):continue
  errors=np.linalg.norm(np.asarray(xy)-world[ci,:2],axis=1);candidates.append((rms,float(errors.max()),R,t,errors))
 if not candidates:raise ValueError('没有满足正深度、实测高度和地面可见性的解；核查点序/单位。')
 candidates.sort(key=lambda c:c[0]);rms,err,R,t,errors=candidates[0];transform(R,t)
 if len(candidates)>1 and abs(candidates[1][0]-rms)<.1 and np.linalg.norm(candidates[1][3]-t)>.05:raise ValueError('平面外参仍有两解歧义；增加地面点范围并检查点序。')
 good=bool(rms<3 and err<.10);save_yaml(a.out,dict(schema='cquick_extrinsics_v1',verified=good,R_carrier_camera=R.tolist(),t_carrier_camera=t.tolist(),carrier_axes='x_forward_y_left_z_up',camera_axes='x_right_y_down_z_forward',fit_rms_pixels=rms,check_errors_m=errors.tolist(),check_max_error_m=err,measured_height_m=a.measured_height,point_records=records,image_pixels=pixels,reference_image=str(a.image)));print('verified:',good,'height:',t[2],'check max error m:',err)
 if not good:raise ValueError('外参或独立检验未通过，不能用于有效定位。')
def main():
 p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
 c=sub.add_parser('check');c.add_argument('--camera',default='/dev/video0');c.add_argument('--seconds',type=float,default=10);c.add_argument('--out',type=Path,default=Path('results/camera_check'))
 i=sub.add_parser('intrinsic');i.add_argument('--camera',default='/dev/video0');i.add_argument('--cols',type=int,default=9);i.add_argument('--rows',type=int,default=6);i.add_argument('--square-mm',type=float,required=True);i.add_argument('--folder',type=Path,default=Path('calibration/intrinsic_capture'));i.add_argument('--solve-only',action='store_true');i.add_argument('--out',type=Path,default=Path('config/camera_intrinsics.yaml'))
 g=sub.add_parser('ground');g.add_argument('--image',required=True,type=Path);g.add_argument('--points-csv',required=True,type=Path);g.add_argument('--measured-height',required=True,type=float);g.add_argument('--intrinsics',type=Path,default=Path('config/camera_intrinsics.yaml'));g.add_argument('--out',type=Path,default=Path('config/camera_extrinsics.yaml'))
 a=p.parse_args();{'check':check,'intrinsic':intrinsic,'ground':ground}[a.command](a)
if __name__=='__main__':
 try:main()
 except (ValueError,RuntimeError) as e:raise SystemExit(str(e))
