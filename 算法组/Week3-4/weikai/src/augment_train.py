#!/usr/bin/env python3
"""生成含两种干扰球负样本的训练集 soccer_dataset_aug。
- 黄绿 Molten 球（green_patches/）
- 红蓝花纹小球（redblue_patches/）
两者都贴到 train(V1-V3) 图上，标签不变（都不标 = 负样本）。val/test 原样。
"""
import cv2
import numpy as np
import random
import shutil
from pathlib import Path

import sys
src = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("soccer_dataset")
dst = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("soccer_dataset_aug")
if dst.exists():
    shutil.rmtree(dst)
shutil.copytree(src, dst)

def load_patches(d):
    out=[]
    for p in sorted(Path(d).glob("*.png")):
        im=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
        if im is not None and im.shape[2]==4:
            out.append(im)
    return out

green = load_patches("green_patches")
redblue = load_patches("redblue_patches")
print("黄绿球素材:",len(green),"| 红蓝球素材:",len(redblue))

rng = random.Random(42)
imgdir = dst/"images"/"train"
labdir = dst/"labels"/"train"
n_img=n_green=n_red=0

for img_path in sorted(imgdir.glob("*.jpg")):
    r = rng.random()
    if r > 0.78:      # 22% 保持原样
        continue
    im = cv2.imread(str(img_path))
    if im is None: continue
    H,W = im.shape[:2]
    lab_path = labdir/(img_path.stem+".txt")
    balls=[]
    if lab_path.exists():
        for line in lab_path.read_text(encoding="utf-8").splitlines():
            q=line.split()
            if len(q)<5: continue
            cx,cy,bw,bh=map(float,q[1:5])
            balls.append(((cx-bw/2)*W,(cy-bh/2)*H,(cx+bw/2)*W,(cy+bh/2)*H))
    touched=False
    # 决定贴哪种：一半图贴黄绿，一半贴红蓝，部分两者都贴
    kinds=[]
    if r < 0.39: kinds.append("green")
    if r >= 0.39 or rng.random()<0.25: kinds.append("redblue")
    for kind in kinds:
        pool = green if kind=="green" else redblue
        if not pool: continue
        patch = rng.choice(pool).copy()
        ph,pw=patch.shape[:2]
        target = rng.randint(70,150)
        scale=target/max(ph,pw)
        nw,nh=max(20,int(pw*scale)),max(20,int(ph*scale))
        patch=cv2.resize(patch,(nw,nh),interpolation=cv2.INTER_LINEAR)
        ang=rng.uniform(-40,40)
        M=cv2.getRotationMatrix2D((nw/2,nh/2),ang,1.0)
        patch=cv2.warpAffine(patch,M,(nw,nh),flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT,borderValue=(0,0,0,0))
        ah,aw=patch.shape[:2]
        for _ in range(80):
            x=rng.randint(0,max(0,W-aw)); y=rng.randint(0,max(0,H-ah))
            cx,cy=x+aw/2,y+ah/2
            clear=True
            for bx0,by0,bx1,by1 in balls:
                if (bx0-aw*0.6<cx<bx1+aw*0.6) and (by0-ah*0.6<cy<by1+ah*0.6):
                    clear=False; break
            if clear:
                roi=im[y:y+ah,x:x+aw]
                alpha=patch[:,:,3:4].astype(np.float32)/255.0
                fg=patch[:,:,:3].astype(np.float32)
                roi[:]=(fg*alpha+roi.astype(np.float32)*(1-alpha)).astype(np.uint8)
                touched=True
                if kind=="green": n_green+=1
                else: n_red+=1
                break
    if touched:
        cv2.imwrite(str(img_path),im)
        n_img+=1

print("增强图片数:",n_img,"/",len(list(imgdir.glob('*.jpg'))))
print("贴入黄绿球:",n_green,"| 贴入红蓝球:",n_red)
for s in ("val","test"):
    print(f"{s} 图片数:",len(list((dst/'images'/s).glob('*.jpg'))))

# 修正 data.yaml path（copytree 保留了原 path）
cfg=dst/"data.yaml"
txt=cfg.read_text(encoding="utf-8")
lines=[]
for ln in txt.splitlines():
    if ln.startswith("path:"):
        lines.append("path: %s" % dst.resolve())
    else:
        lines.append(ln)
cfg.write_text("\n".join(lines)+"\n",encoding="utf-8")
print("data.yaml path 已修正为", dst.resolve())
