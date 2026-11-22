import os
ROOT = r"D:\W34_dataset\ball"
for split in ["train", "val", "test"]:
    lbl_dir = os.path.join(ROOT, "labels", split)
    agg = {}
    for f in sorted(os.listdir(lbl_dir)):
        if not f.endswith(".txt") or f == "classes.txt":
            continue
        clip = f.split("_f")[0]
        p = os.path.join(lbl_dir, f)
        lines = [ln.strip() for ln in open(p, encoding="utf-8") if ln.strip()]
        a = agg.setdefault(clip, {"img": 0, "inst": 0, "empty": 0})
        a["img"] += 1
        if lines:
            a["inst"] += len(lines)
        else:
            a["empty"] += 1
    for clip, a in agg.items():
        print(f"{split}\t{clip}\timg={a['img']}\tinst={a['inst']}\tempty={a['empty']}")
