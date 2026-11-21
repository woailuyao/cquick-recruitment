from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}
SPLITS = ("train", "val", "test")
DEFAULT_NAME = "ball"


# ---------------------------------------------------------------- 小工具
def log(msg: str = "") -> None:
    print(msg, flush=True)


def _utf8_stdout() -> None:

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")  # type: ignore[attr-defined]
        except Exception:
            pass


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        sys.exit("[错误] 缺少 pyyaml，请先执行：pip install pyyaml")
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def dump_yaml(cfg: dict, path: Path) -> None:
    try:
        import yaml
    except ImportError:
        sys.exit("[错误] 缺少 pyyaml，请先执行：pip install pyyaml")
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def as_list(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(v) for v in value]
    return [str(value)]


def resolve_root(cfg: dict, yaml_path: Path) -> Path:
    """解析 ball_data.yaml 里的 path 字段，相对路径按 yaml 所在目录算。"""
    raw = cfg.get("path")
    if raw in (None, "", "."):
        return yaml_path.parent.resolve()
    root = Path(str(raw)).expanduser()
    if not root.is_absolute():
        root = (yaml_path.parent / root).resolve()
    return root


# ---------------------------------------------------------------- 数据集体检
def scan_split(root: Path, split: str, image_dirs: list[str], nc: int) -> dict:
    """统计一个 split 的图片、标签、框和格式错误。"""
    st = {
        "split": split,
        "images": 0,
        "labeled": 0,
        "empty": 0,      # 有 txt 但内容为空 = 负样本，允许
        "missing": 0,    # 没有 txt = 当负样本用，允许但会提示
        "boxes": 0,
        "cls": Counter(),
        "bad": [],       # 硬错误：格式不对 / 类别超范围
        "warn": [],      # 软提示：框稍微出界
        "stems": [],
    }
    label_dir = root / "labels" / split

    for rel in image_dirs:
        img_dir = (root / rel).resolve()
        if not img_dir.exists():
            st["warn"].append(f"{split} 图片目录不存在：{img_dir}")
            continue
        for img in sorted(img_dir.rglob("*")):
            if not img.is_file() or img.suffix.lower() not in IMG_EXTS:
                continue
            st["images"] += 1
            st["stems"].append(img.stem)
            label = label_dir / f"{img.stem}.txt"
            if not label.exists():
                st["missing"] += 1
                continue

            lines = [
                ln.strip()
                for ln in label.read_text(encoding="utf-8", errors="ignore").splitlines()
                if ln.strip()
            ]
            if not lines:
                st["empty"] += 1
                continue

            st["labeled"] += 1
            for ln in lines:
                parts = ln.split()
                if len(parts) != 5:
                    st["bad"].append(f"{label.name}: 这一行不是 5 个字段 -> {ln!r}")
                    continue
                try:
                    cid = int(float(parts[0]))
                    x, y, w, h = (float(p) for p in parts[1:])
                except ValueError:
                    st["bad"].append(f"{label.name}: 有非数字 -> {ln!r}")
                    continue
                if cid < 0 or cid >= nc:
                    st["bad"].append(f"{label.name}: class_id={cid} 超出 0~{nc - 1} -> {ln!r}")
                    continue
                if w <= 0 or h <= 0:
                    st["bad"].append(f"{label.name}: 宽或高 <= 0 -> {ln!r}")
                    continue
                if not all(0.0 <= v <= 1.0 for v in (x, y, w, h)):
                    st["bad"].append(f"{label.name}: 坐标不在 0~1 之间（忘了归一化？） -> {ln!r}")
                    continue
                if (x - w / 2 < -1e-3 or x + w / 2 > 1 + 1e-3
                        or y - h / 2 < -1e-3 or y + h / 2 > 1 + 1e-3):
                    st["warn"].append(f"{label.name}: 框超出图像边界 -> {ln!r}")

                st["boxes"] += 1
                st["cls"][cid] += 1
    return st


def report(stats: list[dict], nc: int, names: list[str]) -> bool:
    """打印体检报告，返回是否可以开始训练。"""
    log("=" * 68)
    log("数据集体检报告")
    log("=" * 68)
    log(f"{'split':<7}{'图片':>7}{'有标签':>8}{'空标签':>8}{'缺标签':>8}{'框数':>8}")
    for st in stats:
        log(f"{st['split']:<7}{st['images']:>7}{st['labeled']:>8}"
            f"{st['empty']:>8}{st['missing']:>8}{st['boxes']:>8}")

    total_boxes = sum(st["boxes"] for st in stats)
    total_imgs = sum(st["images"] for st in stats)
    log("-" * 68)
    log(f"共 {total_imgs} 张图片，{total_boxes} 个标注框，类别数 nc={nc}，names={names}")

    # 类别分布
    dist = Counter()
    for st in stats:
        dist.update(st["cls"])
    if dist:
        parts = [f"{names[c] if c < len(names) else c}={n}" for c, n in sorted(dist.items())]
        log("类别分布：" + "，".join(parts))

    bad_total = sum(len(st["bad"]) for st in stats)
    if bad_total:
        log("-" * 68)
        log(f"[错误] 发现 {bad_total} 处标签格式问题，下面只列前 10 条：")
        shown = 0
        for st in stats:
            for msg in st["bad"]:
                if shown >= 10:
                    break
                log(f"  - {st['split']}: {msg}")
                shown += 1
        log("请先把这些标签修好再训练，否则训练不会报错，但指标会莫名其妙地差。")

    warns = [w for st in stats for w in st["warn"]]
    if warns:
        log("-" * 68)
        log(f"[提示] {len(warns)} 条软提示（不影响训练，下面只列前 5 条）：")
        for w in warns[:5]:
            log(f"  - {w}")

    # 帧泄漏粗查：同名文件同时出现在多个 split
    seen: dict[str, str] = {}
    leak = []
    for st in stats:
        for stem in st["stems"]:
            if stem in seen and seen[stem] != st["split"]:
                leak.append(f"{stem} 同时在 {seen[stem]} 和 {st['split']}")
            seen.setdefault(stem, st["split"])
    if leak:
        log("-" * 68)
        log(f"[警告] 可能有帧泄漏（同名图片出现在不同 split），共 {len(leak)} 组，例如：")
        for item in leak[:5]:
            log(f"  - {item}")
        log("不能把一段视频的相邻帧打散。")

    # 硬性检查
    ok = True
    train = next(s for s in stats if s["split"] == "train")
    val = next(s for s in stats if s["split"] == "val")
    if train["images"] == 0:
        log("[错误] train 集是空的。")
        ok = False
    if val["images"] == 0:
        log("[错误] val 集是空的，训练过程中无法评估。")
        ok = False
    if bad_total:
        ok = False
    if ok:
        log("=" * 68)
        log("体检通过，可以训练。")
    return ok


# ---------------------------------------------------------------- 训练 / 评估
def pick_device(requested: str):
    if requested != "auto":
        return requested if requested == "cpu" else (int(requested) if requested.isdigit() else requested)
    try:
        import torch
        if torch.cuda.is_available():
            log(f"[信息] 检测到 CUDA，使用 GPU: {torch.cuda.get_device_name(0)}")
            return 0
    except ImportError:
        pass
    log("[信息] 没有可用 CUDA，使用 CPU 训练（会慢，建议把 epochs 先设小一点试通）。")
    return "cpu"


def train_and_eval(args, data_yaml: Path, device):
    try:
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        sys.exit(f"[错误] 缺少依赖：{exc}\n请先执行：pip install -U ultralytics")

    log("=" * 68)
    log(f"开始训练：model={args.model}  epochs={args.epochs}  imgsz={args.imgsz}  "
        f"batch={args.batch}  device={device}")
    log(f"data = {data_yaml}")
    log("首次运行会自动下载 yolov8n.pt 预训练权重（约 6 MB），需要联网。")
    log("=" * 68)

    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        exist_ok=True,
        pretrained=True,
        optimizer="auto",
        seed=0,
        deterministic=True,
        patience=args.patience,
        plots=True,
        val=True,
        verbose=True,
    )

    save_dir = Path(model.trainer.save_dir)
    best = Path(model.trainer.best)
    log("=" * 68)
    log(f"训练结束，结果目录：{save_dir}")
    log(f"最佳权重：{best}")

    metrics_lines = []
    if args.no_test:
        log("[跳过] 没有在 test 集上评估（--no-test）。")
    else:
        log("=" * 68)
        log("在独立 test 集上评估……")
        m = model.val(
            data=str(data_yaml),
            split="test",
            imgsz=args.imgsz,
            batch=args.batch,
            device=device,
            project=args.project,
            name=f"{args.name}_test",
            exist_ok=True,
            plots=True,
            verbose=False,
        )
        precision = float(m.box.mp)
        recall = float(m.box.mr)
        map50 = float(m.box.map50)
        map5095 = float(m.box.map)
        log(f"Precision   = {precision:.4f}")
        log(f"Recall      = {recall:.4f}")
        log(f"mAP50       = {map50:.4f}")
        log(f"mAP50-95    = {map5095:.4f}")
        metrics_lines += [
            f"Precision : {precision:.4f}",
            f"Recall    : {recall:.4f}",
            f"mAP50     : {map50:.4f}",
            f"mAP50-95  : {map5095:.4f}",
        ]

    summary = save_dir / "metrics_summary.txt"
    summary.write_text(
        "\n".join([
            "CQuick 第 3-4 周考核 · 训练记录",
            f"时间        : {__import__('datetime').datetime.now():%Y-%m-%d %H:%M:%S}",
            f"模型        : {args.model}",
            f"数据集      : {data_yaml}",
            f"epochs      : {args.epochs}",
            f"imgsz       : {args.imgsz}",
            f"batch       : {args.batch}",
            f"device      : {device}",
            f"torch       : {torch.__version__}",
            f"ultralytics : {__import__('ultralytics').__version__}",
            f"best.pt     : {best}",
            "",
            *metrics_lines,
        ]) + "\n",
        encoding="utf-8",
    )
    log(f"指标已写入：{summary}")

    log("=" * 68)
    log("下一步：拿摄像头做实时检测，例如")
    log(f"  python realtime_ball_detect.py --weights {best} --camera 0")
    log(f"  或者快速看一眼效果：yolo detect predict model={best} source=0 imgsz=640 conf=0.25 show=True")
    log("=" * 68)


# ---------------------------------------------------------------- 入口
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="YOLOv8 单类别训练 + 测试集评估",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--root", default=None, help="数据集根目录（images/ labels/ 的上一层）")
    p.add_argument("--data", default=None, help="ball_data.yaml 路径；不给则由 --root 自动生成")
    p.add_argument("--names", nargs="+", default=[DEFAULT_NAME],
                   help="类别名，任务1就写 ball；任务2写 4 个标志类")
    p.add_argument("--model", default="yolov8n.pt", help="预训练权重")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16, help="显存不够就改 8 或 4")
    p.add_argument("--device", default="auto", help="auto / cpu / 0")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--patience", type=int, default=20, help="多少轮没提升就早停")
    p.add_argument("--project", default="runs/detect")
    p.add_argument("--name", default="ball_yolov8n")
    p.add_argument("--check-only", action="store_true", help="只体检数据集，不训练")
    p.add_argument("--no-test", action="store_true", help="跳过 test 集评估")
    return p


def main() -> None:
    _utf8_stdout()
    args = build_parser().parse_args()
    names = [str(n) for n in args.names]
    nc = len(names)

    # 1. 找到 ball_data.yaml（没有就生成）
    if args.data:
        data_yaml = Path(args.data).expanduser().resolve()
        if not data_yaml.exists():
            sys.exit(f"[错误] 找不到 ball_data.yaml：{data_yaml}")
    else:
        if not args.root:
            sys.exit("[错误] 至少给一个 --root 或 --data，例如：\n"
                     "  python train_ball_yolov8.py --root ./soccer_dataset")
        root = Path(args.root).expanduser().resolve()
        if not root.exists():
            sys.exit(f"[错误] 数据集目录不存在：{root}")
        data_yaml = root / "ball_data.yaml"
        if not data_yaml.exists():
            cfg = {
                "path": str(root),
                "train": "images/train",
                "val": "images/val",
                "test": "images/test",
                "nc": nc,
                "names": names,
            }
            dump_yaml(cfg, data_yaml)
            log(f"[信息] 已自动生成：{data_yaml}")
            log(f"       path={root}")
            log(f"       names={names}  nc={nc}")

    # 2. 读 yaml 并对齐类别
    cfg = load_yaml(data_yaml)
    yaml_names = as_list(cfg.get("names"))
    yaml_nc = int(cfg.get("nc", len(yaml_names) or nc))
    if yaml_names and yaml_names != names:
        log(f"[提示] ball_data.yaml 里的 names={yaml_names}，本次按脚本的 {names} 检查。")
        names, nc = yaml_names, len(yaml_names)
    if yaml_nc != nc:
        log(f"[提示] ball_data.yaml 里 nc={yaml_nc}，names 有 {nc} 个；按 names 为准。")

    root = resolve_root(cfg, data_yaml)
    log(f"[信息] 数据集根目录：{root}")

    # 3. 体检
    splits = {}
    for split in SPLITS:
        rels = as_list(cfg.get(split))
        if not rels:
            continue
        splits[split] = rels
    if "train" not in splits:
        sys.exit("[错误] ball_data.yaml 里没有 train 字段。")
    if "val" not in splits:
        sys.exit("[错误] ball_data.yaml 里没有 val 字段，训练过程中无法验证。")
    if "test" not in splits and not args.no_test:
        log("[提示] ball_data.yaml 里没有 test 字段，本次跳过 test 集评估。")
        args.no_test = True

    stats = []
    for split, rels in splits.items():
        if split == "test" and args.no_test:
            continue
        stats.append(scan_split(root, split, rels, nc))

    ok = report(stats, nc, names)
    if not ok:
        sys.exit(1)
    if args.check_only:
        log("（--check-only，到此结束，没有开始训练）")
        return

    # 4. 训练 + 评估
    device = pick_device(args.device)
    train_and_eval(args, data_yaml, device)


if __name__ == "__main__":
    main()
