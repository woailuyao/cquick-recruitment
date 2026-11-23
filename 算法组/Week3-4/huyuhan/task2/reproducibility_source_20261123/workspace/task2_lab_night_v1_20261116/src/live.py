import argparse,hashlib,json,math,time
from pathlib import Path
from datetime import datetime
import cv2,numpy as np,yaml
from geometry import NAMES,invalid,transform,read_map,pixel_ground,match,model_class_names_ok
from refine_v3 import refine_v3 as refine
from calibration import camera
from ros_payload import pose_message,ball_message

def configuration(folder):
 folder=Path(folder);p=[folder/n for n in ['camera_intrinsics.yaml','camera_extrinsics.yaml','field.yaml','field_geometry.yaml']]
 if not all(f.is_file() for f in p):return None,'calibration_or_map_missing'
 try:
  intr,ext,field=map(lambda f:yaml.safe_load(f.read_text()),[p[0],p[1],p[3]]);sha=hashlib.sha256(p[2].read_bytes()).hexdigest()
  if not all(x.get('verified') is True for x in [intr,ext,field]):return None,'configuration_not_verified'
  field_data=yaml.safe_load(p[2].read_text(encoding='utf-8'))
  if not isinstance(field_data,dict) or field_data.get('verified') is False or field_data.get('usage')=='reference_draft_only_not_verified_for_live_localization':return None,'configuration_not_verified'
  if field.get('units')!='m' or field.get('axes') not in ['x_right_y_up_yaw_ccw','x_to_goal_y_left_z_up_yaw_ccw'] or field.get('field_sha256')!=sha:raise ValueError('Map units/axes/SHA256 mismatch')
  R,t=transform(ext['R_carrier_camera'],ext['t_carrier_camera']);markers=read_map(p[2]);display=field_data.get('display_geometry',{})
  for line in display.get('polylines',[]):
   points=np.asarray(line['points'],float)
   if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all():raise ValueError('Invalid field display line')
  return dict(intr=intr,R=R,t=t,markers=markers,map_sha256=sha,axes=field['axes'],display_geometry=display),'ok'
 except (ValueError,TypeError,KeyError,yaml.YAMLError) as e:return None,'configuration_invalid: '+str(e)
def process(im,dets,cfg,status,diagnostics=None):
 if diagnostics is None:diagnostics={}
 by_class={NAMES[i]:sum(d['class_id']==i for d in dets) for i in range(len(NAMES))}
 diagnostics.update(raw_detection_count=len(dets),raw_by_class=by_class,
                    refinement_success=0,refinement_reject=0,
                    projection_success=0,projection_reject=0,
                    deduplicated_reject=0,observations=0,rejects=[])
 if cfg is None:
  pose=invalid(status);diagnostics['match_status']=pose['status'];return pose,[]
 intr=cfg['intr']
 if [im.shape[1],im.shape[0]]!=[intr['width'],intr['height']]:
  pose=invalid('resolution_mismatch');diagnostics['match_status']=pose['status'];return pose,[]
 obs=[]
 for d in sorted(dets,key=lambda d:-d['confidence']):
  uv=refine(im,d['bbox'],d['class_id'])
  if uv is None:
   diagnostics['refinement_reject']+=1
   diagnostics['rejects'].append(dict(stage='refinement',class_id=d['class_id'],bbox=d['bbox'],confidence=d['confidence']))
   continue
  diagnostics['refinement_success']+=1
  xy=pixel_ground([uv],intr['camera_matrix'],intr['dist_coeffs'],cfg['R'],cfg['t'])[0]
  if xy is None:
   diagnostics['projection_reject']+=1
   diagnostics['rejects'].append(dict(stage='ground_ray',class_id=d['class_id'],bbox=d['bbox'],pixel=uv))
   continue
  diagnostics['projection_success']+=1
  if any(np.linalg.norm(np.asarray(o['xy'])-xy)<.08 for o in obs):
   diagnostics['deduplicated_reject']+=1
   diagnostics['rejects'].append(dict(stage='deduplication',class_id=d['class_id'],bbox=d['bbox'],pixel=uv))
   continue
  obs.append(dict(class_id=d['class_id'],xy=xy.tolist(),pixel=uv,confidence=d['confidence']))
 pose=match(obs,cfg['markers'],cfg['R'],cfg['t'])
 diagnostics.update(observations=len(obs),match_status=pose['status'],
                    match_count=pose['marker_count'],match_residual_m=pose['residual'])
 return pose,obs

def map_view_coordinates(points,axes):
 points=np.asarray(points,float)
 if axes=='x_to_goal_y_left_z_up_yaw_ccw':return np.stack([-points[...,1],points[...,0]],axis=-1)
 return points

def draw_map(cfg,pose):
 canvas=np.full((720,640,3),35,np.uint8)
 if cfg is None:cv2.putText(canvas,'Map/calibration missing or unverified',(12,190),0,.5,(0,200,255),1);return canvas
 xy=map_view_coordinates([m['xy'] for m in cfg['markers']]+[[0,0]],cfg.get('axes'));lo=xy.min(0)-.5;hi=xy.max(0)+.5;scale=min(580/(hi[0]-lo[0]),450/(hi[1]-lo[1]))
 def pix(p):
  q=map_view_coordinates(p,cfg.get('axes'));return(int(30+(q[0]-lo[0])*scale),int(680-(q[1]-lo[1])*scale))
 for line in cfg.get('display_geometry',{}).get('polylines',[]):
  points=np.array([pix(p) for p in line['points']],np.int32).reshape(-1,1,2);cv2.polylines(canvas,[points],bool(line.get('closed',False)),(200,220,200),2,cv2.LINE_AA)
 if cfg.get('display_geometry',{}).get('scope')=='measured_positive_x_half':cv2.putText(canvas,'MEASURED HALF: +X up, +Y left',(12,205),0,.48,(160,220,160),1)
 origin=pix([0,0]);cv2.arrowedLine(canvas,origin,pix([.5,0]),(200,200,200),2);cv2.arrowedLine(canvas,origin,pix([0,.5]),(200,200,200),2)
 counts={}
 for m in cfg['markers']:
  cls=m['class_id'];counts[cls]=counts.get(cls,0)+1;label=['L','T','X','P'][cls]+str(counts[cls]);pt=pix(m['xy']);cv2.circle(canvas,pt,4,(40,230,130),-1);cv2.putText(canvas,label,(pt[0]+5,pt[1]-5),0,.4,(220,220,220),1)
 if pose['valid']:
  p=np.array([pose['x_camera_map'],pose['y_camera_map']]);angle=math.radians(pose['yaw_camera_map']);cv2.circle(canvas,pix(p),8,(0,160,255),-1);cv2.arrowedLine(canvas,pix(p),pix(p+.45*np.array([math.cos(angle),math.sin(angle)])),(0,160,255),3)
 return canvas

def detect(model,im,device,conf):
 if model is None:return []
 result=model.predict(im,imgsz=640,conf=conf,device=device,verbose=False)[0];out=[]
 if result.boxes is not None:
  for box,cls,score in zip(result.boxes.xyxy.cpu().tolist(),result.boxes.cls.cpu().tolist(),result.boxes.conf.cpu().tolist()):out.append(dict(bbox=box,class_id=int(cls),confidence=float(score)))
 return out

def compose_view(rgb,map_canvas,ball_only=False):
 return rgb if ball_only else np.hstack([rgb,cv2.resize(map_canvas,(640,rgb.shape[0]))])

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',default='/dev/video0');p.add_argument('--offline',action='store_true');p.add_argument('--ball-only',action='store_true');p.add_argument('--marker-model',type=Path);p.add_argument('--ball-model',type=Path);p.add_argument('--config',type=Path,default=Path('config'));p.add_argument('--out',type=Path,default=Path('results/live'));p.add_argument('--device',default='0');p.add_argument('--conf',type=float,default=.25);p.add_argument('--headless',action='store_true');p.add_argument('--max-frames',type=int,default=0);p.add_argument('--test-point',default='unassigned');p.add_argument('--ros2',action='store_true');a=p.parse_args();iscamera=a.source.isdigit() or a.source.startswith('/dev/video') or a.source.startswith('bridge:')
 if not iscamera and not a.offline:raise SystemExit('视频回放必须显式使用--offline，不能标为实时相机验收。')
 if a.ball_only and a.ball_model is None:raise SystemExit('--ball-only需要通过--ball-model指定任务一权重。')
 if a.ball_only and a.marker_model is not None:raise SystemExit('--ball-only模式不加载场地模型。')
 a.out.mkdir(parents=True,exist_ok=False);print('胡钰涵 huyuhan',datetime.now().astimezone().isoformat(),flush=True);print('INPUT:',('LIVE CAMERA' if iscamera else 'OFFLINE REPLAY'),a.source,flush=True);cfg,status=(None,'not_applicable_ball_only') if a.ball_only else configuration(a.config);print('geometry:',status,flush=True);marker=ball=None
 if a.marker_model or a.ball_model:
  from ultralytics import YOLO
  for f in [a.marker_model,a.ball_model]:
   if f is not None and not f.is_file():raise SystemExit('权重不存在：'+str(f))
  marker=YOLO(str(a.marker_model)) if a.marker_model else None;ball=YOLO(str(a.ball_model)) if a.ball_model else None
  if marker and not model_class_names_ok(marker.names):raise SystemExit('场地模型类别/编号不符合LCross/TCross/XCross/PenaltyPoint。')
  if ball and ball.names!={0:'ball'}:raise SystemExit('足球模型必须只有0 ball。')
 if marker is None and not a.ball_only:print('No marker model: preview only; pose remains invalid.',flush=True)
 cap=camera(a.source) if iscamera else cv2.VideoCapture(a.source)
 if not cap.isOpened():raise SystemExit('无法打开输入。')
 node=None;rclpy=None
 if a.ros2:
  import rclpy
  from std_msgs.msg import String
  rclpy.init();node=rclpy.create_node('cquick_field_vision');posepub=node.create_publisher(String,'/localization/field_pose',10);ballpub=node.create_publisher(String,'/soccer/ball',10)
 def publish(packet,balls):
  if node:
   msg=String();msg.data=json.dumps(pose_message(packet),allow_nan=False);posepub.publish(msg);msg=String();msg.data=json.dumps(ball_message(packet,balls),allow_nan=False);ballpub.publish(msg);rclpy.spin_once(node,timeout_sec=0)
 frames=validframes=0;failure_status_saved=set();start=time.perf_counter()
 try:
  with (a.out/'frames.jsonl').open('w',encoding='utf-8',buffering=1) as log:
   while True:
    t0=time.perf_counter();ok,im=cap.read();read_ms=(time.perf_counter()-t0)*1000
    if not ok:
     if iscamera:
      packet=dict(**invalid('camera_read_failed'),timestamp=datetime.now().astimezone().isoformat(),frame=frames,source='camera',test_point=a.test_point);log.write(json.dumps(packet,allow_nan=False)+'\n');publish(packet,[])
     break
    frames+=1;stamp=datetime.now().astimezone().isoformat();t0=time.perf_counter();dets=detect(marker,im,a.device,a.conf);balls=detect(ball,im,a.device,a.conf);infer_ms=(time.perf_counter()-t0)*1000;t0=time.perf_counter();stage={};pose,obs=process(im,dets,cfg,status,stage) if marker else (invalid('not_applicable_ball_only' if a.ball_only else 'marker_model_missing'),[]);loc_ms=(time.perf_counter()-t0)*1000;validframes+=int(pose['valid']);elapsed=time.perf_counter()-start;fps=frames/max(elapsed,1e-9);ballout=[]
    if hasattr(cap,'freshness'):
     fresh,fresh_status=cap.freshness()
     if not fresh:
      validframes-=int(pose['valid']);pose=invalid(fresh_status);dets=[];balls=[];obs=[];stage=dict(stale_frame=True,raw_detection_count=0,refinement_success=0,projection_success=0,observations=0,match_status=fresh_status)
    for d in balls:
     x1,y1,x2,y2=d['bbox'];u,v=(x1+x2)/2,(y1+y2)/2;ballout.append(dict(class_name='ball',confidence=d['confidence'],bbox=d['bbox'],center_u=u,center_v=v));cv2.rectangle(im,(int(x1),int(y1)),(int(x2),int(y2)),(255,150,0),2);cv2.circle(im,(int(u),int(v)),4,(255,150,0),-1);cv2.putText(im,f"ball {d['confidence']:.2f} center=({int(u)},{int(v)})",(int(x1),max(15,int(y1)-3)),0,.5,(255,150,0),1)
    for d in dets:
     x1,y1,x2,y2=map(int,d['bbox']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,255,255),2);cv2.putText(im,f"{NAMES[d['class_id']]} {d['confidence']:.2f}",(x1,max(15,y1-3)),0,.5,(0,255,255),1)
    for i,o in enumerate(obs):
     u,v=map(int,o['pixel']);cv2.circle(im,(u,v),4,(0,0,255),-1);ident=next((m['map_id'] for m in pose['matches'] if m['observation']==i),'unmatched');cv2.putText(im,ident,(u+4,v+14),0,.4,(0,0,255),1)
    packet=dict(timestamp=stamp,frame=frames,source='camera' if iscamera else 'offline_video',test_point=a.test_point,width=im.shape[1],height=im.shape[0],loop_fps=fps,read_ms=read_ms,inference_ms=infer_ms,localization_ms=loc_ms,valid_pose_update_hz=validframes/max(elapsed,1e-9),observations=obs,diagnostics=stage,**pose);packet.update(cap.provenance() if hasattr(cap,'provenance') else dict(source_transport='direct_camera' if iscamera else 'offline_file'));log.write(json.dumps(dict(**packet,raw_marker_detections=dets,ball=ballout),allow_nan=False)+'\n');publish(packet,ballout)
    cv2.putText(im,('LIVE CAMERA' if iscamera else 'OFFLINE REPLAY')+f' f={frames} loopFPS={fps:.1f} infer={infer_ms:.1f}ms',(12,28),0,.6,(0,255,0) if iscamera else (0,0,255),2);canvas=draw_map(cfg,pose);lines=[stamp,f"valid={pose['valid']} status={pose['status']}",f"markers={pose['marker_count']} rms_m={pose['residual']}",f"X={pose['x_camera_map']} Y={pose['y_camera_map']}",f"yaw_deg={pose['yaw_camera_map']}",f"loc_ms={loc_ms:.1f} loopFPS={fps:.1f}"]
    for i,line in enumerate(lines):cv2.putText(canvas,line,(12,25+25*i),0,.47,(255,255,255),1)
    joined=compose_view(im,canvas,a.ball_only)
    if a.ball_only:cv2.putText(joined,stamp,(12,54),0,.55,(0,255,0),1)
    if frames==1:cv2.imwrite(str(a.out/'first_frame_diagnostic.jpg'),joined)
    if not pose['valid'] and pose['status'] not in failure_status_saved and len(failure_status_saved)<12:
     safe_status=''.join(c if c.isalnum() or c=='_' else '_' for c in pose['status'])[:48]
     cv2.imwrite(str(a.out/f'failure_{safe_status}_frame{frames}.jpg'),joined);failure_status_saved.add(pose['status'])
    if not a.headless:
     cv2.imshow('CQuick RGB | field map',joined)
     if cv2.waitKey(1)&255==ord('q'):break
    if a.max_frames and frames>=a.max_frames:break
 finally:
  cap.release()
  if not a.headless:cv2.destroyAllWindows()
  if node:node.destroy_node();rclpy.shutdown()
  summary=dict(source=a.source,input_kind='camera' if iscamera else 'offline_video',frames=frames,valid_frames=validframes,elapsed_s=time.perf_counter()-start,geometry_status=status,ball_only=a.ball_only,marker_model=str(a.marker_model),ball_model=str(a.ball_model),note='loopFPS includes reading, inference, localization, logging and display; capture-only FPS measured separately. Diagnostic image is not acceptance recording.');(a.out/'run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
