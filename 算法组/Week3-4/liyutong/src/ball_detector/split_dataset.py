# -*- coding: utf-8 -*-
"""
数据集划分 —— 防泄漏红线：按视频段划分 train/val/test，绝不随机打散单帧！

用法：python split_dataset.py
前提：raw/<视频段名>/*.jpg + 同名 .txt（labelImg YOLO 格式）
输出：soccer_dataset/{images,labels}/{train,val,test} + data.yaml

分配策略：整段作为最小单位，按张数从大到小贪心分配到「当前最缺量」的集合，
使三个集合尽量接近 7:2:1，同时避免某段被拆开（拆开就是数据泄漏）。
"""
import os
import shutil

RAW = "raw"
DST = "soccer_dataset"
ARCHIVE = "_历史抽帧"
RATIO = (0.7, 0.2, 0.1)
SPLITS = ["train", "val", "test"]


def main():
    if not os.path.isdir(RAW):
        print(f"请先建 {RAW}/ 目录，按视频段分子文件夹放抽帧图")
        return
    segs = []
    for s in sorted(os.listdir(RAW)):
        d = os.path.join(RAW, s)
        if not os.path.isdir(d):
            continue
        imgs = [f for f in os.listdir(d) if f.lower().endswith((".jpg", ".png"))]
        # 只统计有非空标注的图
        with_lbl = [f for f in imgs
                    if os.path.isfile(os.path.join(d, os.path.splitext(f)[0] + ".txt"))
                    and os.path.getsize(os.path.join(d, os.path.splitext(f)[0] + ".txt")) > 0]
        segs.append((s, len(with_lbl), len(imgs)))
        if len(with_lbl) < len(imgs):
            print(f"  ⚠️ {s}: {len(imgs) - len(with_lbl)} 张没有有效标注，将跳过")

    if len(segs) < 5:
        print(f"⚠️ 只有 {len(segs)} 段视频，考核要求至少 5 段")
    total = sum(n for _, n, _ in segs)
    print(f"共 {len(segs)} 段、{total} 张有标注图")

    # 段数不多（6 段 = 3^6 = 729 种），直接穷举出最接近 7:2:1 的组合
    # 约束：每个集合至少 1 段；整段不拆开
    import itertools
    best, best_cost = None, float("inf")
    for combo in itertools.product(range(3), repeat=len(segs)):
        if len(set(combo)) < 3:          # 有空集合，跳过
            continue
        cnt = [0, 0, 0]
        for i, c in enumerate(combo):
            cnt[c] += segs[i][1]
        cost = sum(abs(cnt[k] - total * RATIO[k]) / (total * RATIO[k]) for k in range(3))
        if cost < best_cost:
            best_cost, best = cost, combo
    assign = {sp: [] for sp in SPLITS}
    for i, c in enumerate(best):
        assign[SPLITS[c]].append((segs[i][0], segs[i][1]))
    print(f"穷举 {3 ** len(segs)} 种组合，最接近 7:2:1 的方案偏差 {best_cost:.3f}")

    # 归档旧结果，避免新旧混在一起（不删除）
    if os.path.isdir(DST):
        os.makedirs(ARCHIVE, exist_ok=True)
        bak = os.path.join(ARCHIVE, "soccer_dataset_旧")
        i = 1
        while os.path.exists(bak):
            bak = os.path.join(ARCHIVE, f"soccer_dataset_旧{i}")
            i += 1
        shutil.move(DST, bak)
        print(f"旧结果已移到 {bak}")

    for sp in SPLITS:
        os.makedirs(f"{DST}/images/{sp}", exist_ok=True)
        os.makedirs(f"{DST}/labels/{sp}", exist_ok=True)

    for sp in SPLITS:
        for seg, _ in assign[sp]:
            d = os.path.join(RAW, seg)
            imgs = [f for f in os.listdir(d) if f.lower().endswith((".jpg", ".png"))]
            for f in imgs:
                lbl = os.path.splitext(f)[0] + ".txt"
                lp = os.path.join(d, lbl)
                if not (os.path.isfile(lp) and os.path.getsize(lp) > 0):
                    continue
                shutil.copy(os.path.join(d, f), f"{DST}/images/{sp}/{seg}_{f}")
                shutil.copy(lp, f"{DST}/labels/{sp}/{seg}_{lbl}")

    print("\n划分结果（段级隔离，无泄漏）：")
    for sp in SPLITS:
        c = len(os.listdir(f"{DST}/images/{sp}"))
        segs_in = ", ".join(f"{s}({n})" for s, n in assign[sp])
        pct = c / total * 100 if total else 0
        print(f"  {sp:<5}: {c:3d} 张 ({pct:4.1f}%)  <- {segs_in}")

    with open(f"{DST}/data.yaml", "w", encoding="utf-8") as f:
        f.write(f"path: {os.path.abspath(DST)}\n"
                "train: images/train\nval: images/val\ntest: images/test\n"
                "nc: 1\nnames: ['ball']\n")
    print(f"\n已生成 {DST}/data.yaml（nc: 1, names: ['ball']）")
    print("下一步：python train_ball.py")


if __name__ == "__main__":
    main()
