"""Build labelled result comparison sheets from actual saved detections, not screenshots."""
from pathlib import Path
import json
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs'/'real_summary'
OUT.mkdir(parents=True,exist_ok=True)
def read(p):
    im=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    if im is None: raise ValueError(p)
    return im
def tile(p,title,subtitle):
    im=read(p); h,w=im.shape[:2]; scale=min(640/w,460/h)
    im=cv2.resize(im,(round(w*scale),round(h*scale)))
    canvas=np.full((540,660,3),245,np.uint8)
    y=75+(460-im.shape[0])//2;x=(660-im.shape[1])//2
    canvas[y:y+im.shape[0],x:x+im.shape[1]]=im
    cv2.putText(canvas,title,(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(30,30,30),2)
    cv2.putText(canvas,subtitle,(12,55),cv2.FONT_HERSHEY_SIMPLEX,.49,(40,40,40),1)
    return canvas
def save(name,im):
    ok,buf=cv2.imencode('.png',im);assert ok;(OUT/name).write_bytes(buf.tobytes())
records=json.loads((ROOT/'outputs/real_selected/results.json').read_text())['results']
tiles=[]
for i,r in enumerate(records,1):
    name=Path(r['input']).stem
    tiles.append(tile(ROOT/f'outputs/real_selected/{i:06d}_{name}_annotated.png',name, f"detected={r['detected']} u={r['u']:.2f} v={r['v']:.2f} area={r['area']:.0f}"))
blank=np.full((540,660,3),245,np.uint8)
for j,text in enumerate(['liyutong - real photo results','HSV: (5,200,80)-(25,255,255)','Minimum area: 100; morphology: none','Generated result sheet, not a screenshot','Images: user uploads; original scale detection','Same-color shadows remain a limitation']):
    cv2.putText(blank,text,(15,70+j*55),cv2.FONT_HERSHEY_SIMPLEX,.52,(30,30,30),1)
tiles.append(blank)
save('five_photo_results.png',np.vstack([np.hstack(tiles[:3]),np.hstack(tiles[3:])]))
base=ROOT/'outputs/real_experiment_01'
rows=[]
for label,name,index,a,b in [('Saturation comparison','background',3,'baseline','saturation_B'),('Area comparison','small',2,'baseline','area_B')]:
    row=[]
    for variant in [a,b]:
        rr=json.loads((base/variant/'results.json').read_text())['results'][index-1]
        row.append(tile(base/variant/f'{index:06d}_{name}_annotated.png',label+' / '+variant,f"detected={rr['detected']} u={rr['u']:.1f} v={rr['v']:.1f} area={rr['area']:.0f}"))
    rows.append(np.hstack(row))
save('parameter_comparisons.png',np.vstack(rows))
print(OUT)
