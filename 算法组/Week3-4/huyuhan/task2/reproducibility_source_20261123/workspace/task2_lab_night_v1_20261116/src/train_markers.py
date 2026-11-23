"""Runs real GPU training locally; no fabricated training metrics in this package."""
import argparse,hashlib,json
from pathlib import Path
from datetime import datetime
import yaml
from audit_dataset import audit
NAMES=['LCross','TCross','XCross','PenaltyPoint']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def evaluate(model,data,split,out,device,support):
 metrics=model.val(data=str(data),split=split,imgsz=640,device=device,plots=False,project=str(out),name=split,exist_ok=False,verbose=False);indices=list(map(int,metrics.box.ap_class_index));per={}
 for cls,name in enumerate(NAMES):
  n=support.get(name,0)
  if n==0 or cls not in indices:per[name]=dict(instances=n,precision=None,recall=None,map50=None,map50_95=None,status='no_ground_truth' if n==0 else 'metric_unavailable');continue
  P,R,A,B=metrics.box.class_result(indices.index(cls));per[name]=dict(instances=n,precision=float(P),recall=float(R),map50=float(A),map50_95=float(B),status='evaluated')
 return dict(split=split,per_class=per,map50=float(metrics.box.map50),map50_95=float(metrics.box.map),note='Native AP and selected native P/R operating point; not fixed-.25 P/R, not detection confidence. Classes with no ground truth are null, never overall mAP.')
def main():
 p=argparse.ArgumentParser();p.add_argument('--weights',type=Path);p.add_argument('--epochs',type=int,default=60);p.add_argument('--batch',type=int,default=8);p.add_argument('--device',default='0');p.add_argument('--ack-initial-labels',action='store_true');a=p.parse_args();root=Path(__file__).resolve().parents[1]
 if not a.ack_initial_labels:raise SystemExit('当前为67张初标试验集；仅作试训练请传--ack-initial-labels，不能称为最终验收模型。')
 report=audit(root/'dataset');project=Path.home()/'Cquick/Week3-4';weights=a.weights or next((f for f in [project/'pretrained_coco/yolov8n.pt',project/'yolov8n.pt'] if f.is_file()),None)
 if weights is None or not weights.is_file():raise SystemExit('未找到本机官方yolov8n.pt，请用--weights指定；不自动联网重试下载，不使用任务一best.pt代替COCO初始化。')
 from ultralytics import YOLO
 run=root/'results'/('marker_pilot_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));run.mkdir(parents=True);config=yaml.safe_load((root/'dataset/data.yaml').read_text());config['path']=str((root/'dataset').resolve());data=run/'runtime_data.yaml';data.write_text(yaml.safe_dump(config,sort_keys=False));plan=dict(dataset_manifest_sha256=sha(root/'dataset/manifest.json'),pretrained_sha256=sha(weights),epochs=a.epochs,imgsz=640,batch=a.batch,amp=False,plots=False,seed=42,stage='pilot_initial_labels_not_final',selection_rule='Single YOLOv8n pilot; best by training validation fitness; test after freeze');(run/'experiment_plan.json').write_text(json.dumps(plan,indent=2));(run/'dataset_audit.json').write_text(json.dumps(report,indent=2));model=YOLO(str(weights));model.train(data=str(data),epochs=a.epochs,imgsz=640,batch=a.batch,device=a.device,workers=2,seed=42,deterministic=True,patience=20,project=str(run),name='train',exist_ok=False,amp=False,plots=False)
 best=Path(model.trainer.best)
 if not best.is_file():raise SystemExit('未生成best.pt，停止评估。')
 chosen=YOLO(str(best));val=evaluate(chosen,data,'val',run,a.device,report['split_summary']['val']['instances']);frozen=dict(best=str(best.resolve()),best_sha256=sha(best),validation=val,plan=plan,test_not_used_for_selection=True);(run/'selection_before_test.json').write_text(json.dumps(frozen,indent=2));test=evaluate(chosen,data,'test',run,a.device,report['split_summary']['test']['instances']);(run/'test_metrics.json').write_text(json.dumps(test,indent=2));(root/'latest_marker_model.json').write_text(json.dumps(dict(best=str(best.resolve()),sha256=sha(best),stage='pilot_initial_labels_not_final',run=str(run)),indent=2));print('实际训练评估已完成：',run);print('best.pt:',best)
if __name__=='__main__':main()
