# -*- coding: utf-8 -*-
"""
实拍视频采集（细则 1.1 / 2.4）

为什么单独做个工具：
    考核要求"至少 5 段独立视频"、"记录摄像头接口/设备节点/分辨率/实际帧率/拍摄日期"，
    并且后面要写 dataset_summary.md。如果只在文件管理器里点录像，
    元数据会丢、片段会重名、帧率靠猜。所以录像时就把这些都记下来。

用法：

    # 足球数据（在场地/实验室拍球）
    python scripts/collect_video.py --camera 1 --label ball

    # 场地标志数据（在 C0 场地拍 L/T/X/点球点）
    python scripts/collect_video.py --camera 1 --label marker

    # 无人值守录一段固定时长（测试用，或"对着拍 30 秒"）
    python scripts/collect_video.py --camera 1 --label ball --auto-seconds 30

按键：
    ENTER / SPACE   开始 / 停止录制
    q / ESC         退出
    录制中再按任意键不会误触，只有 ENTER/SPACE 才停

输出：
    recordings/
      ├── ball_20261108_161230.mp4      视频
      ├── ball_20261108_161230.json     元数据（设备/分辨率/实测帧率/时长/帧数）
      ├── ball_20261108_161230.jpg      首帧缩略图
      └── sessions.csv                  所有片段的汇总表（写 dataset_summary 用）
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import platform
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
OUT_ROOT = os.path.join(ROOT, "recordings")


def open_cam(index, width, height, backend="msmf", max_scan=3):
    """打开相机；指定的号打不开时自动扫描其他号（插拔后编号会变）。"""
    flags = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW, "any": cv2.CAP_ANY}
    order = [backend] + [b for b in ("msmf", "dshow", "any") if b != backend]

    def attempt(idx):
        for name in order:
            cap = cv2.VideoCapture(idx, flags.get(name, cv2.CAP_MSMF))
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            ok, _ = cap.read()
            if not ok:
                cap.release()
                continue
            return cap, name, idx
        return None, None, idx

    cap, name, idx = attempt(index)
    if cap is not None:
        return cap, name, idx

    for alt in range(max_scan + 1):
        if alt == index:
            continue
        cap, name, idx = attempt(alt)
        if cap is not None:
            print(f"[相机] index={index} 打不开，已回退到 index={alt}")
            return cap, name, idx
    return None, None, index


def measure_fps(cap, n=45):
    """按连续取帧真实耗时测帧率（CAP_PROP_FPS 不可信）。"""
    t0 = time.perf_counter()
    got = 0
    for _ in range(n):
        ok, _ = cap.read()
        if not ok:
            break
        got += 1
    dt = time.perf_counter() - t0
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0) if False else None
    return (got / dt) if dt > 0 else 0.0


def append_summary(row, path):
    exists = os.path.isfile(path)
    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            w.writeheader()
        w.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--label", default="ball", choices=["ball", "marker", "other"],
                    help="这段视频用来做什么")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--backend", default="msmf")
    ap.add_argument("--outdir", default=OUT_ROOT)
    ap.add_argument("--note", default="", help="场景备注，会写进元数据，例如 '室内灯光_球距3m'")
    ap.add_argument("--auto-seconds", type=float, default=0.0,
                    help=">0 时自动录这么多秒然后退出（无人值守）")
    args = ap.parse_args()

    if args.camera is None:
        try:
            import run as _run
            args.camera = _run.default_camera_index()
        except Exception:
            args.camera = 1

    os.makedirs(args.outdir, exist_ok=True)
    summary_csv = os.path.join(args.outdir, "sessions.csv")

    cap, backend, real_idx = open_cam(args.camera, args.width, args.height, args.backend)
    if cap is None:
        print(f"[错误] 打不开相机 index={args.camera}")
        return 1

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_drv = cap.get(cv2.CAP_PROP_FPS)
    fps_real = measure_fps(cap)

    note = args.note          # 可现场按 n 键修改
    print("=" * 68)
    print(f"相机      : index={real_idx} (请求 {args.camera}) backend={backend}")
    print(f"分辨率    : {aw}x{ah}（请求 {args.width}x{args.height}）")
    print(f"帧率      : 实测 {fps_real:.2f} fps   驱动报告 {fps_drv:.1f}（不可信）")
    print(f"标签      : {args.label}    备注: {note or '-'}")
    print(f"输出目录  : {args.outdir}")
    print("=" * 68)
    if (aw, ah) != (args.width, args.height):
        print(f"[警告] 实际分辨率不是 1280x720，细则要求统一 1280x720")

    note = args.note          # 可用 n 键自定义（ASCII），或数字键选预设场景
    writer = None
    rec_path = None
    rec_start = 0.0
    n_frames = 0
    recording = False
    seg_index = 0
    first_frame = None

    # 场景预设：数字键 1~9 一键标注。
    # 为什么用数字键而不是让现场打字：中文要走输入法，而 OpenCV 窗口不支持 IME，
    # 想把中文敲进终端又得来回切窗口焦点（第一次就是这么失败的）。
    PRESETS = {
        "ball": [
            ("static_left_right", "球静止_左中右"),
            ("rolling_blur", "球滚动_运动模糊"),
            ("far_small", "球远处小目标"),
            ("occluded", "球部分遮挡"),
            ("near_white_line", "贴白线_困难负样本"),
            ("low_view", "低视角"),
            ("top_view", "俯视"),
            ("multi_target", "多目标"),
            ("misc", "其他"),
        ],
        "marker": [
            ("single_t_near_far", "单个T型_近到远"),
            ("l_x_sweep", "L型X型扫视"),
            ("penalty_point", "点球点"),
            ("multi_marker", "多标志同框"),
            ("occluded_edge", "遮挡_边缘"),
            ("backlight", "逆光_阴影"),
            ("far_small", "远处小标志"),
            ("closeup", "近距特写"),
            ("misc", "其他"),
        ],
        "other": [("misc", "其他")],
    }
    presets = PRESETS.get(args.label, PRESETS["other"])
    scene_key, scene_desc = "", ""
    editing = False
    edit_buf = ""

    print("场景快捷键：")
    for i, (k, d) in enumerate(presets, start=1):
        print(f"  {i}  {k:<20} {d}")

    def stop_recording():
        nonlocal writer, recording, rec_path, n_frames, first_frame
        if writer is not None:
            writer.release()
            writer = None
        if rec_path and n_frames > 0:
            dur = time.perf_counter() - rec_start
            base = os.path.splitext(rec_path)[0]
            if first_frame is not None:
                cv2.imwrite(base + ".jpg", first_frame)
            meta = {
                "file": os.path.basename(rec_path),
                "label": args.label,
                "camera_index": args.camera,
                "backend": backend,
                "resolution": [aw, ah],
                "fps_measured": round(fps_real, 2),
                "fps_driver_reported": round(float(fps_drv), 2),
                "frames": n_frames,
                "duration_s": round(dur, 2),
                "avg_fps_in_file": round(n_frames / dur, 2) if dur > 0 else 0,
                "datetime": datetime.datetime.now().isoformat(timespec="seconds"),
                "host": platform.node(),
                "note": note,
                "scene": scene_key,
                "scene_desc": scene_desc,
                "opencv": cv2.__version__,
            }
            with open(base + ".json", "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
            append_summary({
                "datetime": meta["datetime"], "label": args.label,
                "file": meta["file"], "frames": n_frames,
                "duration_s": meta["duration_s"],
                "fps_avg": meta["avg_fps_in_file"],
                "resolution": f"{aw}x{ah}",
                "camera": f"idx{args.camera}", "note": note,
                "scene": scene_key,
                "scene_desc": scene_desc,
            }, summary_csv)
            print(f"\n[完成] {os.path.basename(rec_path)}  "
                  f"{n_frames} 帧 / {dur:.1f}s / 平均 {meta['avg_fps_in_file']:.1f} fps")
            print(f"       元数据 {os.path.basename(base)}.json")
        recording = False
        n_frames = 0

    def start_recording():
        nonlocal writer, rec_path, rec_start, n_frames, recording, first_frame, seg_index
        seg_index += 1
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        rec_path = os.path.join(args.outdir, f"{args.label}_{ts}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(rec_path, fourcc, max(10.0, fps_real), (aw, ah))
        if not writer.isOpened():
            print(f"[错误] 无法写视频: {rec_path}")
            writer = None
            return
        recording = True
        rec_start = time.perf_counter()
        n_frames = 0
        first_frame = None
        print(f"\n[录制中] {os.path.basename(rec_path)}  再按 ENTER/SPACE 停止")

    auto_mode = args.auto_seconds > 0
    if auto_mode:
        print(f"[自动] 开始录制 {args.auto_seconds:.0f} 秒 ...")
        start_recording()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[警告] 取帧失败，结束")
                break

            if recording:
                writer.write(frame)
                if first_frame is None:
                    first_frame = frame.copy()
                n_frames += 1
                if auto_mode and (time.perf_counter() - rec_start) >= args.auto_seconds:
                    stop_recording()
                    break

            # 预览
            vis = frame.copy()
            h, w = vis.shape[:2]
            cv2.rectangle(vis, (0, 0), (w, 56), (0, 0, 0), -1)
            if recording:
                el = time.perf_counter() - rec_start
                cv2.circle(vis, (26, 28), 10, (0, 0, 255), -1)
                cv2.putText(vis, f"REC  {el:5.1f}s   frames={n_frames}",
                            (46, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(vis, f"READY   label={args.label}   "
                                 f"实际 {fps_real:.1f} fps   {aw}x{ah}",
                            (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # 场景行
            scene_txt = f"scene[{scene_key or '-'}] {scene_desc or note or '-'}"
            cv2.putText(vis, scene_txt, (12, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 230, 255), 1, cv2.LINE_AA)
            cv2.putText(vis, f"{len([f for f in os.listdir(args.outdir) if f.endswith('.mp4')])} segments"
                             f"   ENTER=录/停   1-9=场景   n=英文备注   m=中文备注   q=退出",
                        (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

            # 内置文本编辑器（英文/拼音；中文请按 m 走终端）
            if editing:
                cv2.rectangle(vis, (0, h // 2 - 60), (w, h // 2 + 18), (0, 0, 0), -1)
                cv2.putText(vis, "ASCII note only (ENTER=ok  ESC=cancel)  -  for Chinese press m instead",
                            (12, h // 2 - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(vis, edit_buf + "_", (12, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("collect video", vis)
            key = cv2.waitKey(1) & 0xFF

            # 编辑模式优先处理，避免 Enter/q 被误当成录停/退出
            if editing:
                if key in (13, 10):
                    if edit_buf:
                        note = edit_buf
                        scene_key, scene_desc = "custom", edit_buf
                        print(f"[备注] {note}")
                    editing = False
                    edit_buf = ""
                elif key == 27:
                    editing = False
                    edit_buf = ""
                elif key in (8, 127):
                    edit_buf = edit_buf[:-1]
                elif 32 <= key < 127:
                    edit_buf += chr(key)
                continue

            if key in (27, ord("q")):
                break
            if key == ord("n"):
                # 窗口内输入框：只能收 ASCII（英文/数字/下划线）。
                # OpenCV 的键盘事件拿不到输入法候选字，中文打不进去。
                editing = True
                edit_buf = ""
                continue
            if key == ord("m"):
                # 中文备注走终端。必须切到终端窗口才能打字——
                # 键盘输入只会送到"当前有焦点的窗口"。
                print("\n" + "-" * 50)
                print("在下面输入备注（支持中文），输完按回车。")
                print("输完记得用鼠标点回预览窗口，否则按键会失效。")
                print("-" * 50)
                try:
                    newnote = input("备注: ").strip()
                    if newnote:
                        note = newnote
                        print(f"[备注] {note}")
                except Exception:
                    pass
                print("（现在请点回预览窗口）")
                continue
            if ord("1") <= key <= ord("9"):
                idx_preset = key - ord("1")
                if idx_preset < len(presets):
                    scene_key, scene_desc = presets[idx_preset]
                    note = scene_key
                    print(f"[场景] {scene_key}  {scene_desc}")
                continue
            if key in (13, 10, 32):
                if recording:
                    stop_recording()
                else:
                    start_recording()
    finally:
        if recording:
            stop_recording()
        cap.release()
        cv2.destroyAllWindows()

    print("\n" + "=" * 68)
    mp4s = sorted(f for f in os.listdir(args.outdir) if f.endswith(".mp4"))
    print(f"{args.outdir} 下共有 {len(mp4s)} 段视频")
    for f in mp4s:
        print("  ", f)
    print(f"\n汇总表: {summary_csv}")
    if args.label == "ball":
        print("细则要求：足球数据 >= 5 段独立视频，>= 200 张有效标注图")
    else:
        print("细则要求：标志数据 >= 5 段独立视频，>= 300 张有效标注图")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
