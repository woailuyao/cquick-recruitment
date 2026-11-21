"""Two trained YOLO models + explicitly relative planar localization.

Camera input is LIVE; all file input is labelled OFFLINE REPLAY. Saved replay
videos are development results, never continuous live-camera exam evidence.
"""
from pathlib import Path
import argparse,csv,json,math,os,time
from datetime import datetime
os.environ.setdefault('YOLO_CONFIG_DIR',str(Path.cwd()))
os.environ.setdefault('YOLO_AUTOINSTALL','false')
import cv2
import numpy as np
from ultralytics import YOLO, settings
settings.update({'sync':False,'wandb':False,'comet':False,'clearml':False,'mlflow':False,'neptune':False,'dvc':False,'hub':False,'raytune':False})
from relative_localization import RelativeLocalizer

def stamp():return datetime.now().astimezone().isoformat(timespec='milliseconds')
def put(im,text,xy,color=(240,240,240),scale=.5):
    cv2.putText(im,str(text),xy,cv2.FONT_HERSHEY_SIMPLEX,scale,(0,0,0),3,cv2.LINE_AA)
    cv2.putText(im,str(text),xy,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)

def predict(model,frame,args,imgsz=640):
    r=model.predict(frame,imgsz=imgsz,conf=args.conf,device=args.device,verbose=False)[0]
    return [{'class_name':r.names[int(c)],'confidence':float(s),'bbox':[float(v) for v in b],
             'u':float((b[0]+b[2])/2),'v':float((b[1]+b[3])/2)}
            for b,s,c in zip(r.boxes.xyxy.cpu().numpy(),r.boxes.conf.cpu().numpy(),r.boxes.cls.cpu().numpy())]

def map_panel(localizer,pose,scale=None):
    panel=np.full((720,480,3),(36,35,29),np.uint8)
    put(panel,'VIDEO-ESTIMATED RELATIVE MAP',(15,30),(80,220,255),.6)
    put(panel,'1 unit = selected rectangle width',(15,57))
    if scale is not None:
        put(panel,f'Video estimate: {scale:.1f} m per unit',(15,82),(80,220,255))
    # Maps use X to the right, Y upward; all geometry drawn in identical units.
    def pixel(x,y):return (int(65+(x+.8)*200),int(340-y*200))
    cv2.arrowedLine(panel,pixel(-.8,0),pixel(1.0,0),(150,150,150),1)
    cv2.arrowedLine(panel,pixel(0,-.6),pixel(0,.8),(150,150,150),1)
    put(panel,'+X goal',pixel(.7,-.12));put(panel,'+Y left',pixel(.04,.73))
    cv2.circle(panel,pixel(0,0),round(localizer.geometry['circle_radius_units']*200),(95,95,95),1)
    for p in localizer.landmarks:
        xy=pixel(p.x,p.y);cv2.circle(panel,xy,5,(40,225,255),-1)
        put(panel,p.id,(xy[0]+8,xy[1]-7),scale=.42)
    if pose['relative_valid']:
        x,y=pose['x_camera_map_units'],pose['y_camera_map_units'];yaw=math.radians(pose['yaw_camera_map_deg'])
        pt=pixel(x,y);end=pixel(x+.2*math.cos(yaw),y+.2*math.sin(yaw))
        cv2.circle(panel,pt,6,(60,230,80),-1);cv2.arrowedLine(panel,pt,end,(60,230,80),2)
        label = f'Est. X {x*scale:+.2f} m    Y {y*scale:+.2f} m' if scale is not None else f'X={x:.3f} units Y={y:.3f} units'
        put(panel,label,(15,550),scale=.66)
        put(panel,f'Yaw={math.degrees(yaw):.1f} deg',(15,575))
    else:put(panel,'No accepted current pose',(15,550),(80,170,255))
    put(panel,'relative_valid='+str(pose['relative_valid']),(15,607))
    put(panel,'matched='+str(pose['marker_count'])+' residual='+str(None if pose['residual_units'] is None else round(pose['residual_units'],4)),(15,633))
    put(panel,pose['status'],(15,660),scale=.43)
    put(panel,'Fixed camera height / tilt assumed',(15,694),scale=.45)
    return panel

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',required=True,help='camera index, e.g. 0, or video file')
    ap.add_argument('--ball-weights',required=True,type=Path);ap.add_argument('--marker-weights',required=True,type=Path)
    ap.add_argument('--geometry',type=Path,default=Path('config/field_geometry.yaml'))
    ap.add_argument('--field',type=Path,default=Path('config/field.yaml'))
    ap.add_argument('--output',type=Path,default=Path('outputs/demo'))
    ap.add_argument('--device',default='0');ap.add_argument('--conf',type=float,default=.25)
    ap.add_argument('--marker-imgsz',type=int,default=960)
    ap.add_argument('--headless',action='store_true');ap.add_argument('--max-frames',type=int,default=0)
    ap.add_argument('--stride',type=int,default=1);ap.add_argument('--receipt-csv',type=Path)
    ap.add_argument('--ros2',action='store_true',help='Publish JSON messages; needs sourced ROS2 and rclpy')
    ap.add_argument('--estimated-scale', type=Path, help='Optional video-derived scale configuration')
    args=ap.parse_args()
    scale = None
    if args.estimated_scale:
        import yaml
        scale = float(yaml.safe_load(args.estimated_scale.read_text(encoding='utf-8'))['estimated_metres_per_unit'])
        if not math.isfinite(scale) or scale <= 0:
            ap.error('Estimated scale must be positive and finite')
    if args.stride<1 or not 0<args.conf<1:ap.error('Invalid stride/confidence')
    for p in (args.ball_weights,args.marker_weights,args.geometry,args.field):
        if not p.is_file():ap.error('File missing: '+str(p))
    live=args.source.isdigit();mode='LIVE CAMERA' if live else 'OFFLINE REPLAY'
    receipts={}
    if args.receipt_csv:
        with args.receipt_csv.open(encoding='utf-8-sig',newline='') as f:
            receipts={int(r['clip_frame_index'])-1:r['receipt_iso_local'] for r in csv.DictReader(f)
                      if r['clip_file']==Path(args.source).name and r['clip_frame_index']}
    localizer=RelativeLocalizer(args.geometry,args.field)
    ball=YOLO(str(args.ball_weights));marker=YOLO(str(args.marker_weights))
    publisher=None
    if args.ros2:
        from ros2_output import RosOutput
        publisher=RosOutput()
    cap=cv2.VideoCapture(int(args.source),cv2.CAP_DSHOW if os.name=='nt' else cv2.CAP_V4L2) if live else cv2.VideoCapture(args.source)
    if live:
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);cap.set(cv2.CAP_PROP_FPS,30)
    if not cap.isOpened():raise RuntimeError('Cannot open input: '+args.source)
    fps=float(cap.get(cv2.CAP_PROP_FPS));fps=fps if math.isfinite(fps) and fps>0 else 30
    args.output.mkdir(parents=True,exist_ok=True)
    # Refuse to overwrite existing run evidence.
    if (args.output/'records.jsonl').exists():raise FileExistsError('Choose a new --output directory')
    writer=None;count=0;index=-1;valid_count=0;detection_frames=0;times=[];start=stamp();wall=time.perf_counter()
    stop='end_of_file';records=(args.output/'records.jsonl').open('w',encoding='utf-8')
    try:
        while True:
            tick=time.perf_counter();ok=cap.grab();index+=1
            if not ok:
                if live:stop='camera_read_failed'
                break
            if index%args.stride:continue
            ok,raw=cap.retrieve()
            if not ok:raise RuntimeError('Frame decode failed at '+str(index))
            receipt=stamp()
            balls=predict(ball,raw,args);marks=predict(marker,raw,args,args.marker_imgsz)
            pose=localizer.estimate(marks,raw)
            if scale is not None:
                valid = pose['relative_valid']
                pose.update(estimated_x_camera_map_m=pose['x_camera_map_units'] * scale if valid else None,
                            estimated_y_camera_map_m=pose['y_camera_map_units'] * scale if valid else None,
                            estimated_yaw_camera_map_deg=pose['yaw_camera_map_deg'] if valid else None)
            elapsed=time.perf_counter()-tick;times.append(elapsed)
            count+=1;valid_count+=int(pose['relative_valid']);detection_frames+=int(bool(balls))
            throughput=1/np.mean(times[max(1,len(times)-30):]) if len(times)>1 else 1/elapsed
            record={'processed_at':stamp(),'name':'kuaizhenhua','input_mode':mode,
                    'source':str(args.source) if live else Path(args.source).name,
                    'frame_index_0based':index,'source_time_seconds':None if live else index/fps,
                    'source_receipt_iso':receipt if live else receipts.get(index),
                    'processing_ms':elapsed*1000,'ball_detected':bool(balls),'balls':balls,'markers':marks,'pose':pose}
            records.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            if publisher:publisher.publish(record)
            vis=raw.copy()
            for det in balls+marks:
                x1,y1,x2,y2=np.rint(det['bbox']).astype(int)
                color=(30,240,60) if det['class_name']=='ball' else (255,180,40)
                cv2.rectangle(vis,(x1,y1),(x2,y2),color,2)
                put(vis,det['class_name']+f" {det['confidence']:.2f}",(x1,max(18,y1-6)),color,.55)
                cv2.drawMarker(vis,(round(det['u']),round(det['v'])),color,cv2.MARKER_CROSS,10,1)
            for obs in pose['observations']:cv2.drawMarker(vis,tuple(np.rint(obs['pixel']).astype(int)),(0,0,255),cv2.MARKER_CROSS,16,2)
            vis=cv2.resize(vis,(1280,720));cv2.rectangle(vis,(0,0),(1280,108),(25,25,25),-1)
            put(vis,'kuaizhenhua | '+mode+' | processed '+record['processed_at'],(12,25),(100,240,255),.65)
            put(vis,f'Frame {index} | processing throughput {throughput:.1f} fps | {elapsed*1000:.1f} ms',(12,52))
            put(vis,'Source receipt: '+str(record['source_receipt_iso'])+' | source FPS metadata: '+f'{fps:.3f}',(12,77))
            put(vis,'No ball' if not balls else f"Ball detections: {len(balls)}  |  first centre (u,v) = ({balls[0]['u']:.1f}, {balls[0]['v']:.1f}) px",(12,101))
            screen=np.hstack((vis,map_panel(localizer,pose,scale)))
            if writer is None:
                writer=cv2.VideoWriter(str(args.output/'annotated.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps/args.stride,(1760,720))
                if not writer.isOpened():raise RuntimeError('Cannot create video')
            writer.write(screen)
            if count==1 or (pose['relative_valid'] and valid_count==1):
                cv2.imencode('.jpg',screen)[1].tofile(str(args.output/('first_frame.jpg' if count==1 else 'first_relative_pose.jpg')))
            if not args.headless:
                cv2.imshow('Week3-4 Vision | Q quit',screen)
                if cv2.waitKey(1)&255 in (ord('q'),27):stop='user_quit';break
            if args.max_frames and count>=args.max_frames:stop='max_frames';break
    finally:
        records.close();cap.release()
        if writer:writer.release()
        cv2.destroyAllWindows()
        if publisher:publisher.close()
    total=time.perf_counter()-wall
    summary={'started_at':start,'finished_at':stamp(),'input_mode':mode,'frames_processed':count,
             'ball_detection_frames':detection_frames,'relative_pose_frames':valid_count,'metric_pose_frames':0,
             'wall_seconds':total,'end_to_end_processed_fps':count/total,
             'mean_processing_ms_excluding_first':float(np.mean(times[1:])*1000) if len(times)>1 else None,
             'source_fps_metadata':fps,'stride':args.stride,'stop_reason':stop,
             'performance_note':'Offline throughput is not measured live camera FPS; first inference includes warm-up.'}
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=True,indent=2))
if __name__=='__main__':main()
