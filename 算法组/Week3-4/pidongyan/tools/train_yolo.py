"""训练 + 独立测试集评估（考核基线参数已内置）。

默认参数就是考核基线：yolov8n.pt / imgsz=640 / epochs=50 / batch=auto / device=0。
任何与基线不同的参数都会打印出来，并写入 runs/detect/<name>/train_args.md，
方便直接抄进 README（细则要求「训练参数」字段如实填写）。

用法：
  python tools/train_yolo.py                                  # 考核基线（原行为不变）
  python tools/train_yolo.py --preset small_ball --name train_v2   # 小目标改进版
  python tools/train_yolo.py --epochs 120 --imgsz 960 --mosaic 0 --name custom
  python tools/train_yolo.py --eval-only --model runs/detect/train_v2/weights/best.pt --imgsz 960
  python tools/train_yolo.py --data config/marker.yaml --name markers

结束后会：
  1) 在测试集上跑 val，打印 P / R / mAP50 / mAP50-95
  2) 把本次参数与指标追加到 logs/train_runs.csv（多轮对比）
  3) 打印 best.pt 的绝对路径，供 detect_realtime.py 使用

设计原则：best.pt 一律由 val 集选出，test 集只用于最终报告，
不拿 test 指标挑 checkpoint / 调超参（避免测试集泄漏）。
"""
import argparse
import csv
import datetime as dt
import os
import shutil
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]

# 考核基线。改这里等于改基线，除非用户明确要求，不要动。
BASELINE = dict(
    model="yolov8n.pt", epochs=50, imgsz=640, batch="auto",
    device="0", workers=4, patience=30, seed=0,
)

# 针对「训练集全是大球、测试集全是小球」的改进预设，依据见
# logs/box_stats_*.txt 与 runs/detect/train/results.csv：
#   * imgsz 960：1280x720 letterbox 到 960 时缩放 0.75，test 的 82px 球
#     在输入里还剩 ~62px；640 时只剩 41px，P3 头上不到 6px。
#   * mosaic=0 / erasing=0：baseline 曲线前 40 epoch 几乎不收敛，
#     close_mosaic 生效后 mAP50 立刻从 0.39 跳到 0.57~0.70，说明这两项
#     对「大球 + 大量空场」数据有害。
#   * epochs 120 + warmup 5 + cos_lr：baseline 前 20 epoch 白跑，等效只训 30 轮。
#   * optimizer=SGD / lr0=0.01：baseline 的 optimizer=auto 会忽略 lr0 并改选
#     AdamW(lr=0.002)，早期 val/cls_loss 一度飙到 3853，显式 SGD 更稳。
PRESETS = {
    "baseline": {},
    "small_ball": dict(
        epochs=120, imgsz=960, patience=40,
        optimizer="SGD", lr0=0.01, lrf=0.01,
        warmup_epochs=5, cos_lr=True,
        mosaic=0.0, close_mosaic=0, erasing=0.0,
        scale=0.6,
    ),
}

# 允许命令行覆盖的 ultralytics 参数（默认 None = 不传，保持基线/预设值）
OVERRIDE_KEYS = ("model", "epochs", "imgsz", "batch", "device", "workers",
                 "patience", "seed", "optimizer", "lr0", "lrf",
                 "warmup_epochs", "cos_lr", "mosaic", "close_mosaic",
                 "erasing", "scale", "cls", "amp", "flipud", "degrees",
                 "hsv_h", "hsv_s", "hsv_v", "translate", "copy_paste")


def check_dataset(data_yaml: Path):
    cfg = yaml.safe_load(data_yaml.read_text(encoding="utf-8"))
    base = Path(cfg.get("path", "."))
    if not base.is_absolute():
        base = (data_yaml.parent / base).resolve()

    print(f"数据集根目录：{base}")
    problems = []
    for split in ("train", "val", "test"):
        rel = cfg.get(split)
        if not rel:
            problems.append(f"配置缺少 {split}")
            continue
        img_dir = base / rel
        if not img_dir.exists():
            problems.append(f"{split} 图片目录不存在：{img_dir}")
            continue
        n_img = sum(1 for p in img_dir.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
        print(f"  {split:5s} {n_img:5d} 张  {img_dir}")
        if n_img == 0:
            problems.append(f"{split} 没有图片")

    print(f"  nc={cfg.get('nc')}  names={cfg.get('names')}")
    if cfg.get("nc") == 1:
        names = cfg.get("names")
        flat = names if isinstance(names, list) else list(names.values())
        if flat and flat[0] != "ball":
            problems.append(f"任务一类别名必须是 ball，当前 {flat}")

    if problems:
        print("\n[问题]")
        for p in problems:
            print("  -", p)
        raise SystemExit("数据集检查未通过，先按 README 第四节准备数据")
    print("数据集检查通过\n")


def build_effective(args):
    """基线 -> 预设 -> 命令行显式覆盖，三层合并。"""
    eff = dict(BASELINE)
    eff.update(PRESETS[args.preset])
    overrides = {k: getattr(args, k) for k in OVERRIDE_KEYS
                 if getattr(args, k, None) is not None}
    eff.update(overrides)
    return eff, overrides


def report_params(eff, overrides, args):
    diff = {k: v for k, v in eff.items() if BASELINE.get(k) != v}
    print("训练参数（与考核基线不同处必须写进 README）：")
    print(f"  preset={args.preset}  name={args.name}")
    print(f"  model={eff['model']} imgsz={eff['imgsz']} epochs={eff['epochs']} "
          f"batch={eff['batch']} device={eff['device']} seed={eff['seed']}")
    if diff:
        print("  ⚠ 非基线项：")
        for k, v in sorted(diff.items()):
            tag = " (命令行)" if k in overrides else " (预设)"
            print(f"      {k} = {v}{tag}")
    else:
        print("  与基线完全一致，无需在 README 额外说明")
    return diff


def save_args_md(run_dir: Path, eff, diff, args, metrics=None):
    run_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        f"# 训练记录 {args.name}",
        f"",
        f"- 时间：{dt.datetime.now():%Y-%m-%d %H:%M:%S}",
        f"- preset：`{args.preset}`",
        f"- 数据：`{args.data}`",
        f"- 基线：yolov8n.pt / imgsz=640 / epochs=50 / batch=auto / device=0",
        f"",
        f"## 本次全部参数",
        f"",
        "| 参数 | 值 | 与基线不同 |",
        "|---|---|---|",
    ]
    for k, v in sorted(eff.items()):
        lines.append(f"| {k} | {v} | {'是' if k in diff else '否'} |")
    lines += ["", "## 独立测试集结果", ""]
    if metrics:
        lines += [
            f"| 指标 | 值 |", "|---|---|",
            f"| Precision | {metrics['P']:.4f} |",
            f"| Recall | {metrics['R']:.4f} |",
            f"| mAP50 | {metrics['mAP50']:.4f} |",
            f"| mAP50-95 | {metrics['mAP50_95']:.4f} |",
            "",
            f"> best.pt 由 **val** 集选出，test 集仅用于最终报告，未用于挑模型或调参。",
        ]
    (run_dir / "train_args.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def append_run_log(eff, args, metrics):
    path = ROOT / "logs" / "train_runs.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    row = dict(
        time=f"{dt.datetime.now():%Y-%m-%d %H:%M:%S}", name=args.name,
        preset=args.preset, model=eff["model"], epochs=eff["epochs"],
        imgsz=eff["imgsz"], batch=eff["batch"], optimizer=eff.get("optimizer", "auto"),
        lr0=eff.get("lr0", "auto"), mosaic=eff.get("mosaic", 1.0),
        erasing=eff.get("erasing", 0.4),
        **{k: (f"{v:.4f}" if v is not None else "") for k, v in metrics.items()},
        weights=str(ROOT / "runs" / "detect" / args.name / "weights" / "best.pt"),
    )
    exists = path.exists()
    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            w.writeheader()
        w.writerow(row)
    print(f"本轮指标已追加到 {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(ROOT / "config" / "data.yaml"))
    ap.add_argument("--name", default="train")
    ap.add_argument("--preset", choices=list(PRESETS), default="baseline")
    ap.add_argument("--eval-only", action="store_true", help="不训练，只跑 test 评估")
    ap.add_argument("--no-copy-best", action="store_true",
                    help="不把 best.pt 复制到项目根（多轮对比时避免互相覆盖）")
    for key in OVERRIDE_KEYS:
        if key in ("model", "batch", "device", "optimizer"):
            ap.add_argument(f"--{key.replace('_', '-')}", default=None)
        elif key in ("cos_lr",):
            ap.add_argument(f"--{key.replace('_', '-')}", action="store_true", default=None)
        elif key in ("lr0", "lrf", "mosaic", "erasing", "scale", "cls",
                     "flipud", "degrees", "hsv_h", "hsv_s", "hsv_v",
                     "translate", "copy_paste"):
            ap.add_argument(f"--{key.replace('_', '-')}", type=float, default=None)
        elif key in ("amp",):
            ap.add_argument(f"--{key}", choices=["on", "off"], default=None,
                            help="AMP 混合精度；早期 loss 爆炸时可试 off")
        else:
            ap.add_argument(f"--{key.replace('_', '-')}", type=int, default=None)
    args = ap.parse_args()

    # 之前反复踩坑：在 C:\windows\System32 下运行会找不到相对路径，
    # 且系统 Python 没有 cv2。这里把 cwd 固定到项目根，任何目录调用都一致。
    os.chdir(ROOT)

    data_yaml = Path(args.data)
    if not data_yaml.is_absolute():
        data_yaml = ROOT / data_yaml
    if not data_yaml.exists():
        raise SystemExit(f"找不到 {data_yaml}")
    check_dataset(data_yaml)

    eff, overrides = build_effective(args)
    diff = report_params(eff, overrides, args)
    run_dir = ROOT / "runs" / "detect" / args.name
    save_args_md(run_dir, eff, diff, args)

    from ultralytics import YOLO

    batch = -1 if str(eff["batch"]).lower() in ("auto", "-1") else int(eff["batch"])
    amp = None if eff.get("amp") is None else (eff["amp"] == "on")

    if not args.eval_only:
        train_kw = dict(
            data=str(data_yaml),
            epochs=eff["epochs"], imgsz=eff["imgsz"], batch=batch,
            device=eff["device"], workers=eff["workers"],
            patience=eff["patience"], seed=eff["seed"],
            project=str(ROOT / "runs" / "detect"), name=args.name,
            exist_ok=True, pretrained=True, plots=True,
        )
        for k in ("optimizer", "lr0", "lrf", "warmup_epochs", "cos_lr",
                  "mosaic", "close_mosaic", "erasing", "scale", "cls",
                  "flipud", "degrees", "hsv_h", "hsv_s", "hsv_v",
                  "translate", "copy_paste"):
            if k in eff:
                train_kw[k] = eff[k]
        if amp is not None:
            train_kw["amp"] = amp

        model = YOLO(eff["model"])
        model.train(**train_kw)

    if args.eval_only and "model" in overrides:
        # --eval-only 且显式 --model：直接评估指定权重（常用于复测旧模型）
        best = Path(overrides["model"])
        if not best.is_absolute():
            best = ROOT / best
    else:
        best = run_dir / "weights" / "best.pt"
    if not best.exists():
        raise SystemExit(f"找不到权重 {best}")
    print(f"\n最佳权重：{best}")

    print("\n在独立测试集上评估（best.pt 由 val 选出，test 仅用于报告）：")
    val_kw = dict(data=str(data_yaml), split="test",
                  imgsz=eff["imgsz"], device=eff["device"])
    if amp is not None:
        val_kw["amp"] = amp
    metrics_obj = YOLO(str(best)).val(**val_kw)
    box = metrics_obj.box
    metrics = dict(P=box.mp, R=box.mr, mAP50=box.map50, mAP50_95=box.map)
    print(f"  Precision  {metrics['P']:6.3f}")
    print(f"  Recall     {metrics['R']:6.3f}")
    print(f"  mAP50      {metrics['mAP50']:6.3f}")
    print(f"  mAP50-95   {metrics['mAP50_95']:6.3f}")

    save_args_md(run_dir, eff, diff, args, metrics)
    append_run_log(eff, args, metrics)
    print(f"\n下一步：python tools/detect_realtime.py --source 0 --model {best}")

    # 基线那一轮保持原行为（复制成根目录 best.pt）；其它 name 各存一份，
    # 避免 v2 把 v1 的权重悄悄覆盖掉，导致报告里的指标对不上文件。
    dst = ROOT / ("best.pt" if args.name == "train" else f"best_{args.name}.pt")
    if not args.no_copy_best:
        try:
            shutil.copy2(best, dst)
            print(f"已复制到 {dst}")
        except Exception as e:
            print(f"[提示] 复制 best.pt 失败（不影响训练）：{e}")


if __name__ == "__main__":
    main()
