import sys, re, os, time
from pathlib import Path
import cv2

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

REPO = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("W34_DATA", REPO.parents[4] / "week34_data"))
VIDEO_DIR = DATA / "raw_videos"
OUT_ROOT = DATA / "dataset"
KEEP_FPS = 2.0
SPLIT_RATIO = (0.70, 0.20, 0.10)
JPEG_QUALITY = 95
BALL_NAMES = ["ball"]
MARKER_NAMES = ["LCross", "TCross", "XCross", "PenaltyPoint"]
TASKS = {
    "ball": {"pattern": r"clip0[1-5]", "max_frames": 45, "min_sec": 20.0},
    "marker": {"pattern": r"clip(08|12|14|1[6-9]|2[0-2])", "max_frames": 80, "min_sec": 6.0},
}


def list_clips():
    exts = (".avi", ".mp4", ".mov", ".mkv")
    return sorted(p for p in VIDEO_DIR.iterdir() if p.suffix.lower() in exts)


def clip_meta(clip):
    cap = cv2.VideoCapture(str(clip))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    return fps, n, (n / fps if fps else 0.0)


def pick_clips(all_clips, metas, pattern, min_sec):
    rx = re.compile(pattern)
    chosen = []
    for c in all_clips:
        if not rx.search(c.stem):
            continue
        fps, n, dur = metas[c.name]
        if dur < min_sec:
            print(f"  跳过 {c.name}: 时长 {dur:.1f}s < {min_sec}s，太短的片段相邻帧会污染数据集")
            continue
        chosen.append(c)
    return chosen


def clip_frames(clip, src_fps, max_frames):
    stride = max(1, round(src_fps / KEEP_FPS))
    cap = cv2.VideoCapture(str(clip))
    idx, kept = 0, []
    while len(kept) < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            kept.append((idx, frame))
        idx += 1
    cap.release()
    return kept


def assign_splits(clip_names, nframes, ratio=SPLIT_RATIO):
    total = sum(nframes[c] for c in clip_names)
    targets = {"train": total * ratio[0], "val": total * ratio[1], "test": total * ratio[2]}
    filled = {k: 0.0 for k in targets}
    plan = {}
    for name in sorted(clip_names, key=lambda c: nframes[c], reverse=True):
        choice = min(targets, key=lambda k: filled[k] - targets[k])
        plan[name] = choice
        filled[choice] += nframes[name]
    for k in [s for s in targets if filled[s] == 0]:
        src = max(targets, key=lambda t: filled[t])
        move = min([n for n in plan if plan[n] == src], key=lambda n: nframes[n])
        plan[move] = k
        filled[src] -= nframes[move]
        filled[k] += nframes[move]
    return plan


def write_dataset(task, plan, frames_map):
    counts = {"train": 0, "val": 0, "test": 0}
    for split in counts:
        (OUT_ROOT / task / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT_ROOT / task / "labels" / split).mkdir(parents=True, exist_ok=True)
    for clip_name, split in plan.items():
        stem = Path(clip_name).stem
        for idx, frame in frames_map[clip_name]:
            fp = OUT_ROOT / task / "images" / split / f"{stem}_f{idx:06d}.jpg"
            cv2.imwrite(str(fp), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            counts[split] += 1
    return counts


def write_data_yaml(task, names):
    root = (OUT_ROOT / task).as_posix()
    lines = [f"path: {root}", "train: images/train", "val: images/val", "test: images/test",
             f"nc: {len(names)}", "names:"]
    lines += [f"  {i}: {n}" for i, n in enumerate(names)]
    (OUT_ROOT / task / "data.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_summary(metas, task_plan, task_counts):
    lines = ["# 数据集统计\n",
             f"生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"原始视频目录: {VIDEO_DIR}",
             f"抽帧间隔: {1/KEEP_FPS:.1f} s（KEEP_FPS={KEEP_FPS}）",
             "划分单位: 完整视频片段，同一帧不会跨 split 出现\n",
             "## 片段清单", "",
             "| 片段 | 时长 s | 帧数 | ball | marker |",
             "|---|---:|---:|---|---|"]
    for name, (fps, n, dur) in sorted(metas.items()):
        b = task_plan["ball"].get(name, "-")
        m = task_plan["marker"].get(name, "-")
        lines.append(f"| {name} | {dur:.1f} | {n} | {b} | {m} |")
    lines.append("")
    for task in ("ball", "marker"):
        c = task_counts.get(task)
        if c:
            lines.append(f"**{task}** train/val/test = {c['train']}/{c['val']}/{c['test']}，"
                         f"合计 {sum(c.values())} 张，来自 {len(task_plan[task])} 段独立视频\n")
    lines.append("## 被排除的片段\n")
    for task, cfg in TASKS.items():
        for name, (fps, n, dur) in sorted(metas.items()):
            if re.search(cfg["pattern"], name) and name not in task_plan[task]:
                lines.append(f"- {name} 用于 {task}: 时长 {dur:.1f}s 低于门槛 {cfg['min_sec']}s")
    (OUT_ROOT / "dataset_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    clips = list_clips()
    if not clips:
        raise SystemExit(f"{VIDEO_DIR} 里没有视频")
    metas = {c.name: clip_meta(c) for c in clips}
    print(f"找到 {len(clips)} 段视频")
    task_plan, task_counts = {"ball": {}, "marker": {}}, {}
    for task, cfg in TASKS.items():
        chosen = pick_clips(clips, metas, cfg["pattern"], cfg["min_sec"])
        print(f"\n=== {task}: 采用 {[c.name for c in chosen]}（{len(chosen)} 段，每段上限 {cfg['max_frames']} 帧）")
        if len(chosen) < 5:
            print(f"  !! 只有 {len(chosen)} 段，细则要求 >=5 段独立视频，需要补录")
        frames_map, src_fps = {}, {}
        for c in chosen:
            fps, n, dur = metas[c.name]
            kept = clip_frames(c, fps, cfg["max_frames"])
            if not kept:
                print(f"  !! {c.name} 抽不到帧")
                continue
            frames_map[c.name] = kept
            src_fps[c.name] = fps
            print(f"  {c.name}: {dur:.1f}s / {n} 帧 -> 留 {len(kept)} 帧")
        if not frames_map:
            continue
        nframes = {k: len(v) for k, v in frames_map.items()}
        total = sum(nframes.values())
        if total < (200 if task == "ball" else 300):
            print(f"  !! 合计 {total} 张，低于细则要求（ball>=200, marker>=300）")
        plan = assign_splits(list(frames_map), nframes)
        print("  划分:", plan)
        counts = write_dataset(task, plan, frames_map)
        write_data_yaml(task, BALL_NAMES if task == "ball" else MARKER_NAMES)
        print(f"  写出 {counts} 合计 {sum(counts.values())} 张")
        task_plan[task] = plan
        task_counts[task] = counts
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    write_summary(metas, task_plan, task_counts)
    print("\n汇总写入", OUT_ROOT / "dataset_summary.md")


if __name__ == "__main__":
    main()
