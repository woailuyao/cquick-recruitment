"""Record personal camera data with a per-frame capture timeline; no detector overlays."""
import argparse,csv,json,time
from datetime import datetime
from pathlib import Path
import cv2
from calibration import camera,read

def main():
 p=argparse.ArgumentParser();p.add_argument('--camera',default='/dev/video0');p.add_argument('--seconds',type=float,default=20);p.add_argument('--out',required=True,type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False);cap=camera(a.camera);fps=cap.get(cv2.CAP_PROP_FPS);fps=fps if 1<=fps<=120 else 30.;writer=cv2.VideoWriter(str(a.out/'camera.avi'),cv2.VideoWriter_fourcc(*'MJPG'),fps,(1280,720));start=time.perf_counter();n=0
 if not writer.isOpened():cap.release();raise SystemExit('视频编码器未能打开。')
 try:
  with (a.out/'frame_timestamps.csv').open('w',newline='') as f:
   w=csv.writer(f);w.writerow(['frame','timestamp','elapsed_s'])
   while time.perf_counter()-start<a.seconds:
    im=read(cap);n+=1;stamp=datetime.now().astimezone().isoformat();elapsed=time.perf_counter()-start;writer.write(im);w.writerow([n,stamp,elapsed])
 finally:writer.release();cap.release()
 elapsed=time.perf_counter()-start;r=dict(source=a.camera,frames=n,actual_elapsed_s=elapsed,measured_recording_loop_fps=n/max(elapsed,1e-9),video_container_fps=fps,note='Personal camera capture, fixed-FPS AVI encoding. Use frame_timestamps.csv for actual capture times. Not a screen-recorded acceptance demo.');(a.out/'capture_report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2));print(r)
if __name__=='__main__':main()
