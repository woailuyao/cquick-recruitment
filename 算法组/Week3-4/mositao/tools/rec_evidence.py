# -*- coding: utf-8 -*-
"""录屏自证一屏：姓名拼音 + 实时时间 + 模型信息 + 独立测试集指标 + 分镜清单。

细则第六节：录屏开始时必须先在终端执行命令，让姓名与实时时间出现在画面中
（Windows 下等价：
    echo 姓名: mositao; Get-Date -Format "yyyy-MM-dd HH:mm:ss"
）；细则 1.8 第 4 条又要求录屏里展示训练完成后的模型信息与独立测试集指标。
本脚本把这几样压在同一屏，录屏时开一次即可自证，不必来回翻文件。

用法（项目根目录）：
    .\\run.bat tools\\rec_evidence.py
    .\\run.bat tools\\rec_evidence.py --name mositao --no-wait

注意：本脚本不打开相机，避免与随后要启动的实时程序抢设备。
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR, rel          # noqa: E402

METRICS = OUTPUT_DIR / "metrics_ball.md"


def sha16(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16].upper()


def parse_metrics(path: Path) -> list[tuple[str, str]]:
    """从 metrics_ball.md 的表格里抓 P / R / mAP 四行。"""
    if not path.exists():
        return []
    rows: list[tuple[str, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\|\s*(Precision|Recall|mAP50|mAP50-95)\s*\|\s*([0-9.]+)\s*\|", line)
        if m:
            rows.append((m.group(1), m.group(2)))
    return rows


def model_info(path: Path) -> tuple[list[str], int, int]:
    """返回 (类名列表, 参数量, 层数)。加载失败时返回空信息，不阻断自证。"""
    try:
        from ultralytics import YOLO

        m = YOLO(str(path)).model
        names = [m.names[i] for i in sorted(m.names)] if isinstance(m.names, dict) else list(m.names)
        params = sum(p.numel() for p in m.parameters())
        return names, params, len(list(m.modules()))
    except Exception as exc:                                        # noqa: BLE001
        return [f"<读取失败: {type(exc).__name__}>"], 0, 0


def dataset_info() -> list[str]:
    """数据集规模、类别与按段划分——细则 1.8 第 1~4 条要靠它证明。"""
    ds = ROOT / "dataset"
    yaml_path = ds / "data.yaml"
    if not yaml_path.exists():
        return [f"数据集: 未找到 {rel(yaml_path)}"]
    try:
        import yaml

        cfg = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    except Exception:                                               # noqa: BLE001
        cfg = {}

    lines = [f"数据集: {rel(yaml_path)}  nc={cfg.get('nc')}  names={cfg.get('names')}"]
    seg_re = re.compile(r"(ball_s\d+)")
    parts, total, segs = [], 0, set()
    for split in ("train", "val", "test"):
        files = sorted((ds / "images" / split).glob("*.png"))
        total += len(files)
        cnt: dict[str, int] = {}
        for p in files:
            m = seg_re.search(p.name)
            key = m.group(1) if m else "?"
            cnt[key] = cnt.get(key, 0) + 1
            segs.add(key)
        detail = " / ".join(f"{k} {v}" for k, v in sorted(cnt.items()))
        parts.append(f"{split} {len(files)}" + (f" ({detail})" if detail else ""))

    boxes = 0
    for split in ("train", "val", "test"):
        for p in (ds / "labels" / split).glob("*.txt"):
            boxes += sum(1 for line in p.read_text(encoding="utf-8").splitlines() if line.strip())

    lines.append("        " + "  |  ".join(parts))
    lines.append(f"        合计 {total} 张 / {len(segs)} 段视频 / {boxes} 个标注框，按视频段划分（同段不跨集合）")
    return lines


STORYBOARD = """\
[录屏分镜 · 任务一，约 60–70 秒，一镜到底不要剪]
  1  本屏停留                      5 s   姓名 + 时间 + 模型 + 指标
  2  启动 realtime_ball.py         5 s   窗口出现，HUD 显示设备/分辨率/FPS
  3  球静止在场地中                5 s   绿框 + conf + 中心点
  4  缓慢移动球（远→近）          10 s   不同位置与大小都要有
  5  快速滚动球                    8 s   动态不漏
  6  球移出画面再进来              8 s   短暂丢失后能重新框住
  7  空场 3 秒                     3 s   证明没球时不硬凑框
  8  回终端看逐帧输出              5 s   frame/ts/fps/infer/bbox/center
  9  讲误检与漏检案例             15 s   outputs/cases_ball/case_sheet_*.png
 10  补一句"测试集 49 张，P/R/mAP 见上屏"
"""

UPLOAD_NOTE = """\
[录完以后]
  · 上传 B 站（标题含"姓名拼音 + Week3-4 任务一 足球检测"），权限选"公开"或"仅链接可见"
  · GitHub 只提交链接，不上传录屏原文件（细则：GitHub 只放轻量工程文件）
  · 把链接填进 README 的「B 站演示录屏」一节，并勾选展示项清单
"""


def main() -> int:
    ap = argparse.ArgumentParser(description="录屏自证一屏")
    ap.add_argument("--name", default="mositao", help="姓名拼音（与 GitHub 目录一致）")
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_ball.pt"))
    ap.add_argument("--no-wait", action="store_true", help="打印后直接退出，不等回车")
    args = ap.parse_args()

    wpath = Path(args.weights)
    now = datetime.now()
    bar = "=" * 74

    print(bar)
    print(f"姓名: {args.name}")
    print(f"时间: {now:%Y-%m-%d %H:%M:%S}  (本地时间，UTC{now.astimezone():%z})")
    print(f"主机: {Path.home().name}  项目: {ROOT.name}  脚本: tools/rec_evidence.py")
    print(bar)

    if wpath.exists():
        print(f"模型: {rel(wpath)}")
        print(f"      {wpath.stat().st_size:,} B   SHA256[:16] {sha16(wpath)}")
        names, params, layers = model_info(wpath)
        print(f"      类别 {names}   参数 {params:,}   模块 {layers}")
    else:
        print(f"模型: 缺失 {rel(wpath)}")

    rows = parse_metrics(METRICS)
    if rows:
        print(f"指标: 独立测试集（{rel(METRICS)}，conf 0.25）")
        for k, v in rows:
            print(f"      {k:<9} {v}")
    else:
        print(f"指标: 未找到 {rel(METRICS)}")

    for line in dataset_info():
        print(line)

    print(bar)
    print(STORYBOARD)
    print(bar)
    print(UPLOAD_NOTE)
    print(bar)

    if not args.no_wait:
        try:
            input("按回车开始录屏后的第 2 步（先启动录屏软件，再按回车）…")
        except EOFError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
