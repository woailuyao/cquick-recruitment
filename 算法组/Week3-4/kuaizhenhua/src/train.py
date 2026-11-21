"""Portable training entry. Supply a real local data.yaml, no downloads forced."""
from pathlib import Path
import argparse,os,json,hashlib
from datetime import datetime
os.environ.setdefault('YOLO_CONFIG_DIR',str(Path.cwd()))
os.environ.setdefault('YOLO_AUTOINSTALL','false')
from ultralytics import YOLO
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--kind',choices=['ball','marker'],required=True)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--initial',type=Path,required=True)
    p.add_argument('--epochs',type=int,default=60);p.add_argument('--imgsz',type=int,default=640)
    p.add_argument('--batch',type=int,default=8);p.add_argument('--device',default='0')
    p.add_argument('--output',type=Path,default=Path('runs'));p.add_argument('--name',default='train')
    args=p.parse_args()
    if not args.initial.is_file():p.error('Initial weights do not exist')
    start=datetime.now().astimezone().isoformat()
    model=YOLO(str(args.initial));model.train(data=str(args.data.resolve()),epochs=args.epochs,
        imgsz=args.imgsz,batch=args.batch,device=args.device,workers=0,project=str(args.output),name=args.name,
        seed=42,deterministic=True,amp=False,patience=args.epochs if args.kind=='marker' else 20,
        val=args.kind=='ball',degrees=12 if args.kind=='marker' else 0,fliplr=.5,flipud=0,close_mosaic=10)
    folder=Path(model.trainer.save_dir);best=folder/'weights/best.pt'
    metric=YOLO(str(best)).val(data=str(args.data.resolve()),split='test' if args.kind=='ball' else 'val',
        imgsz=args.imgsz,batch=args.batch,device=args.device,workers=0,project=str(args.output),name=args.name+'_evaluation')
    report={'started_at':start,'finished_at':datetime.now().astimezone().isoformat(),
        'evaluation':'held-out original clip004' if args.kind=='ball' else 'train-only fitting diagnostic',
        'metrics':metric.results_dict,'weights_sha256':hashlib.sha256(best.read_bytes()).hexdigest()}
    (folder/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if __name__=='__main__':main()
