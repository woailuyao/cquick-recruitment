import hashlib,json
from pathlib import Path
from collections import Counter

def audit(folder):
 folder=Path(folder);rows=json.loads((folder/'manifest.json').read_text(encoding='utf-8'));names=set();groups={};expected=set();labels=set()
 for r in rows:
  if r['image'] in names:raise ValueError('Duplicate image ID')
  names.add(r['image']);s=r['split'];g=r['source_group']
  if g in groups and groups[g]!=s:raise ValueError('Source video leakage')
  groups[g]=s;p=folder/r['relative_path'];label=folder/'labels'/s/(Path(r['image']).stem+'.txt');expected.add(p);labels.add(label)
  if p.parent!=folder/'images'/s or hashlib.sha256(p.read_bytes()).hexdigest()!=r['jpeg_sha256']:raise ValueError('Path/hash mismatch '+r['image'])
  lines=label.read_text().splitlines()
  if len(lines)!=len(r['objects']):raise ValueError('Object count mismatch')
  for line,obj in zip(lines,r['objects']):
   cls,*xy=line.split();a,b,c,d=obj['box_xyxy'];truth=[(a+c)/2560,(b+d)/1440,(c-a)/1280,(d-b)/720]
   if len(xy)!=4 or int(cls)!=obj['class_id'] or not(0<=a<c<=1280 and 0<=b<d<=720) or any(abs(float(x)-y)>1e-7 for x,y in zip(xy,truth)):raise ValueError('Invalid label '+r['image'])
 if set((folder/'images').glob('*/*.jpg'))!=expected or set((folder/'labels').glob('*/*.txt'))!=labels:raise ValueError('Unexpected image/label files')
 summary={}
 for s in ['train','val','test']:
  rr=[r for r in rows if r['split']==s]
  if not rr:raise ValueError('Empty split')
  summary[s]=dict(images=len(rr),instances=dict(Counter(o['class_name'] for r in rr for o in r['objects'])))
 return dict(images=len(rows),source_videos=len(groups),split_summary=summary,stage='pilot_initial_labels_not_final',test_xcross_supported=summary['test']['instances'].get('XCross',0)>0)
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('folder',nargs='?',default=str(Path(__file__).resolve().parents[1]/'dataset'));print(json.dumps(audit(p.parse_args().folder),ensure_ascii=False,indent=2))
