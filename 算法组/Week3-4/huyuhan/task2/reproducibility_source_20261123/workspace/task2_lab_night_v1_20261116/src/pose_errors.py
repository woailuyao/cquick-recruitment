"""Offline reference comparison only. Reference values never enter the localizer."""
import argparse,csv,json,math
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--logs',type=Path,required=True);p.add_argument('--reference-csv',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 if a.out.exists():raise SystemExit('输出已存在，请换新文件名，保留历史记录。')
 refs=list(csv.DictReader(a.reference_csv.open(encoding='utf-8-sig')));allrows=[]
 for path in a.logs.rglob('frames.jsonl'):
  allrows.extend(json.loads(line) for line in path.open() if line.strip())
 output=[]
 for r in refs:
  frames=[x for x in allrows if x.get('test_point')==r['test_point'] and x.get('source')=='camera'];valid=[x for x in frames if x.get('valid') is True];record=dict(**r,total_frames=len(frames),valid_frames=len(valid),valid_fraction=len(valid)/len(frames) if frames else 0,position_error_m=None,yaw_error_deg=None)
  if valid:
   x=float(np.median([f['x_camera_map'] for f in valid]));y=float(np.median([f['y_camera_map'] for f in valid]));angles=np.deg2rad([f['yaw_camera_map'] for f in valid]);coherence=float(np.hypot(np.sin(angles).mean(),np.cos(angles).mean()));yaw=math.degrees(math.atan2(np.sin(angles).mean(),np.cos(angles).mean()));record.update(x_est_m=x,y_est_m=y,yaw_est_deg=yaw,yaw_resultant_length=coherence)
   if coherence<.8:record['status']='unstable_yaw_do_not_report_single_pose'
   else:record.update(status='evaluated',position_error_m=math.hypot(x-float(r['x_ref_m']),y-float(r['y_ref_m'])),yaw_error_deg=abs((yaw-float(r['yaw_ref_deg'])+180)%360-180))
  else:record['status']='no_valid_camera_pose'
  output.append(record)
 a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False));print(json.dumps(output,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
