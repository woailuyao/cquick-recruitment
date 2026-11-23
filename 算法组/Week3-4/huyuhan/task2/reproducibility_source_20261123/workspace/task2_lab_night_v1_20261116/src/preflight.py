import importlib,json,sys,glob,os
from pathlib import Path
from datetime import datetime
from audit_dataset import audit
root=Path(__file__).resolve().parents[1]
def main():
 r=dict(timestamp=datetime.now().astimezone().isoformat(),python=sys.executable,python_version=sys.version,video_devices=glob.glob('/dev/video*'),ros_distro=os.environ.get('ROS_DISTRO'),dataset=audit(root/'dataset'),modules={})
 for name in ['cv2','torch','ultralytics','numpy','scipy','yaml']:
  try:
   m=importlib.import_module(name);r['modules'][name]=dict(available=True,version=getattr(m,'__version__','unknown'))
   if name=='torch':r['cuda_available']=m.cuda.is_available();r['gpu']=m.cuda.get_device_name(0) if m.cuda.is_available() else None
  except Exception as e:r['modules'][name]=dict(available=False,error=str(e))
 print(json.dumps(r,ensure_ascii=False,indent=2));d=root/'results/preflight';d.mkdir(parents=True,exist_ok=True);(d/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')).write_text(json.dumps(r,ensure_ascii=False,indent=2))
 if any(not x['available'] for x in r['modules'].values()):raise SystemExit('先核对是否激活原有venv，不要直接重装PyTorch/CUDA。')
if __name__=='__main__':main()
