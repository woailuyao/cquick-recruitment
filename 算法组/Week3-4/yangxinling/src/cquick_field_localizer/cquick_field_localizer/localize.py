#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localize.py —— 命令行主程序：跑视频/摄像头，输出分屏可视化 + jsonl + 统计报告

典型用法（**在仓库根目录执行**：有 config/ calibration/ 的那一层）：

    # 1) 用录好的视频跑一遍（推荐先这样，方便反复调参）
    python -m cquick_field_localizer.localize \
        --source video/场地/A点1.avi --out out/A点1_vis.mp4 --jsonl out/A点1.jsonl

    # 2) 用 USB 摄像头实时跑（现场演示）
    python -m cquick_field_localizer.localize --source 0

    # 3) 只跑前 300 帧、不写视频，只看统计
    python -m cquick_field_localizer.localize --source video/场地/B点.avi --max-frames 300 --no-video

输出：
    --out    分屏录像（左右各半，和任务书要求的 UI 一致）
    --jsonl  每帧一行 JSON（写报告、算误差都靠它）
    --report 跑完打印的统计（也会存成 yaml）

注意：Python 包路径。如果你不在仓库根目录，请先
    export PYTHONPATH=$PWD/src/cquick_field_localizer
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import yaml

from .field_view import FieldView, make_split_view, CLASS_COLORS, STATUS_COLORS
from .pipeline import LocalizerPipeline, detections_to_json


# ---------------------------------------------------------------------------
def draw_left_panel(frame, rec, width=640, height=720):
    """
    左半屏 = 上方"检测叠加画面" + 下方"检测明细表"。

    为什么要把表格放进来：1280×720 的画面塞进 640 宽会留下大片黑边，
    干脆把这块空间用来显示每个检测的类别/置信度/细化后像素/地面距离，
    录屏时评委能一眼看到"确实做了几何细化"，也方便自己调参。
    """
    img = frame.copy()
    for d in rec["detections"]:
        x1, y1, x2, y2 = [int(round(v)) for v in d["bbox"]]
        col = CLASS_COLORS.get(int(d["cls_id"]), (255, 255, 255))
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)

        label = "%s %.2f" % (d["cls_name"], d["conf"])
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(img, (x1, max(0, y1 - th - 6)), (x1 + tw + 6, y1), col, -1)
        cv2.putText(img, label, (x1 + 3, max(10, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)

        u, v = d["uv"]
        cx, cy = (d["bbox"][0] + d["bbox"][2]) / 2.0, (d["bbox"][1] + d["bbox"][3]) / 2.0
        cv2.circle(img, (int(cx), int(cy)), 4, (180, 180, 180), 1)      # bbox 中心
        if d["refine_ok"]:
            cv2.drawMarker(img, (int(round(u)), int(round(v))), col, cv2.MARKER_CROSS, 12, 2)
        else:
            cv2.drawMarker(img, (int(round(u)), int(round(v))), (0, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 8, 1)

        txt = "(%d,%d)" % (int(round(u)), int(round(v)))
        if np.isfinite(d.get("depth_m", float("nan"))):
            txt += " %.2fm" % d["depth_m"]
        cv2.putText(img, txt, (int(u) + 8, int(v) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    # 图像等比缩放贴到上半部分
    h, w = img.shape[:2]
    img_h = int(round(width * 9.0 / 16.0))
    r = min(width / float(w), img_h / float(h))
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    canvas = np.full((height, width, 3), 22, dtype=np.uint8)
    ox, oy = (width - nw) // 2, (img_h - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized

    # 顶部信息条
    bar = "frame %d | %s | FPS %.1f" % (rec["frame_id"], rec["stamp"], rec["fps"])
    cv2.rectangle(canvas, (0, 0), (width, 26), (12, 12, 12), -1)
    cv2.putText(canvas, bar, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (230, 230, 230), 1, cv2.LINE_AA)

    # 明细表
    ty = img_h + 26
    cv2.putText(canvas, "detections (%d)" % len(rec["detections"]), (10, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    ty += 20
    cv2.putText(canvas, "class      conf  (u,v)      px->ground  match(map)      res", (10, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (140, 140, 140), 1, cv2.LINE_AA)
    ty += 15
    for d in rec["detections"][:8]:
        col = CLASS_COLORS.get(int(d["cls_id"]), (255, 255, 255))
        u, v = d["uv"]
        ground = ("%.2fm" % d["depth_m"]) if np.isfinite(d.get("depth_m", float("nan"))) else "horizon"
        pb = d.get("p_body", [float("nan")] * 2)
        body = ("(%.2f,%.2f)" % (pb[0], pb[1])) if np.isfinite(pb[0]) else "(---,---)"
        name = d.get("match_name") or "-"
        mdist = d.get("match_dist_m", float("nan"))
        res = ("%.3f" % mdist) if np.isfinite(mdist) else "-"
        line = "%-9s %.2f (%4d,%4d) %-9s %-13s %-11s %s" % (
            d["cls_name"], d["conf"], int(round(u)), int(round(v)),
            ("line" if d["refine_ok"] else "bbox"), body, name, res)
        cv2.putText(canvas, line, (10, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.33, col, 1, cv2.LINE_AA)
        ty += 14

    # 相机与地图信息块（录屏要求：地图文件/版本、内参、畸变、外参）
    mi = rec.get("map_info") or {}
    ci = rec.get("camera_info") or {}
    if mi or ci:
        by = ty + 6
        cv2.putText(canvas, "map : %s  ver=%s  n=%d" %
                    (mi.get("path", "?"), mi.get("version", "?"), mi.get("count", 0)),
                    (10, by), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 200, 150), 1, cv2.LINE_AA)
        by += 14
        cv2.putText(canvas, "      sha256=%s..." % mi.get("shorthash", "?"),
                    (10, by), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 200, 150), 1, cv2.LINE_AA)
        by += 14
        cv2.putText(canvas, "intr: fx=%.1f fy=%.1f cx=%.1f cy=%.1f" %
                    (ci.get("fx", 0), ci.get("fy", 0), ci.get("cx", 0), ci.get("cy", 0)),
                    (10, by), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170, 190, 210), 1, cv2.LINE_AA)
        by += 14
        cv2.putText(canvas, "dist: k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.1f" %
                    (ci.get("k1", 0), ci.get("k2", 0), ci.get("p1", 0), ci.get("p2", 0),
                     ci.get("k3", 0)),
                    (10, by), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170, 190, 210), 1, cv2.LINE_AA)
        by += 14
        cv2.putText(canvas, "extr: h=%.3f m  pitch=%.2f deg  roll=%.2f deg" %
                    (ci.get("height_m", 0), ci.get("pitch_deg", 0), ci.get("roll_deg", 0)),
                    (10, by), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170, 190, 210), 1, cv2.LINE_AA)

    # 底部状态条
    pose = rec["pose"]
    scol = STATUS_COLORS.get(pose.get("status", "lost"), (200, 200, 200))
    res = pose.get("residual", float("nan"))
    res_txt = ("%.3f m" % res) if np.isfinite(res) else "---"
    line = "valid=%s  markers=%d  residual=%s  status=%s" % (
        pose.get("valid"), pose.get("marker_count", 0), res_txt,
        pose.get("status", "lost"))
    cv2.rectangle(canvas, (0, height - 24), (width, height), (12, 12, 12), -1)
    cv2.putText(canvas, line, (8, height - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                scol, 1, cv2.LINE_AA)
    return canvas


# ---------------------------------------------------------------------------
def open_source(source):
    """source 可以是视频文件、"0"/"1" 摄像头编号、图片目录，或 http://... 网络流。"""
    src = str(source)
    if src.startswith(("http://", "https://")):
        from .netcam import open_source as _open_net
        cap = _open_net(src)
        if not cap.isOpened():
            raise SystemExit("打不开网络视频流 %s" % src)
        return cap, src
    if src.isdigit():
        cap = cv2.VideoCapture(int(source))
        if not cap.isOpened():
            raise SystemExit("打不开摄像头 %s" % source)
        return cap, None
    if os.path.isdir(source):
        raise SystemExit("暂不支持目录，请传视频文件或摄像头编号：%s" % source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit("打不开视频 %s" % source)
    return cap, source


def main(argv=None):
    ap = argparse.ArgumentParser(description="单目场地自定位（任务2）命令行")
    ap.add_argument("--config", default="config/camera.yaml")
    ap.add_argument("--source", default="0", help="视频文件路径，或摄像头编号 0/1/2")
    ap.add_argument("--out", default="", help="输出分屏录像路径（.mp4）；留空则不写")
    ap.add_argument("--jsonl", default="", help="每帧一行 JSON 的输出路径")
    ap.add_argument("--report", default="", help="统计报告 yaml 路径")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧（0=全部）")
    ap.add_argument("--every", type=int, default=1, help="抽帧步长（1=每帧）")
    ap.add_argument("--no-video", action="store_true", help="不写视频（只要 jsonl/统计）")
    ap.add_argument("--no-show", action="store_true", help="不弹窗（跑批处理时用）")
    ap.add_argument("--save-frames", default="", help="把分屏帧另存到该目录（每 N 帧一张）")
    ap.add_argument("--save-every", type=int, default=60, help="--save-frames 的间隔帧数")
    ap.add_argument("--print-every", type=int, default=30, help="控制台进度打印间隔")
    ap.add_argument("--prior", default="",
                    help="初始位姿先验 x,y,yaw_deg（米/度）。场地关于中线镜像对称，\n"
                         "不给先验时首帧必然 ambiguous（两解等价），给了就只保留门内那一个，"
                         "之后靠帧间连续性维持。\n"
                         "注意：坐标带负号时**必须用等号写法** --prior=-2.0,1.0,0 ，"
                         "否则 argparse 会把 -2.0,1.0,0 当成另一个选项而报 "
                         "'expected one argument'。")
    args = ap.parse_args(argv)

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(args.config)))
    pipe = LocalizerPipeline(args.config, repo_root=repo_root)
    print(pipe.describe())
    if args.prior:
        try:
            px, py, pyaw = [float(v) for v in args.prior.split(",")]
        except ValueError:
            raise SystemExit("--prior 需要三个数：x,y,yaw_deg，例如 --prior 1.2,-2.5,39")
        pipe.set_initial_prior(px, py, pyaw)
        print("[cquick] 初始先验: x=%.2f m y=%.2f m yaw=%.1f deg"
              % (px, py, pyaw))

    viewer_cfg = pipe.cfg.get("viewer", {}) or {}
    total_w = int(viewer_cfg.get("width", 1280))
    total_h = int(viewer_cfg.get("height", 720))
    fview = FieldView(pipe.field, pipe.geometry,
                      width=total_w // 2, height=total_h,
                      pad_m=float(viewer_cfg.get("field_pad_m", 0.6)))

    cap, path = open_source(args.source)
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    writer = None
    if args.out and not args.no_video:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.out, fourcc, src_fps, (total_w, total_h))
        if not writer.isOpened():
            print("[警告] 打不开 VideoWriter，改为不写视频")
            writer = None

    jf = None
    if args.jsonl:
        os.makedirs(os.path.dirname(os.path.abspath(args.jsonl)), exist_ok=True)
        jf = open(args.jsonl, "w", encoding="utf-8")

    if args.save_frames:
        os.makedirs(args.save_frames, exist_ok=True)

    frame_id = 0
    processed = 0
    t_start = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if args.every > 1 and (frame_id % args.every) != 0:
                frame_id += 1
                continue
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            rec = pipe.process(frame, frame_id, stamp)

            left = draw_left_panel(frame, rec, width=total_w // 2, height=total_h)
            pose_view = dict(rec["pose"])
            pose_view.setdefault("processing_ms", rec.get("processing_ms", 0.0))
            right = fview.render(pose=pose_view, observations=rec["obs_map"],
                                 status=rec["pose"].get("status", "lost"),
                                 fps=rec["fps"], frame_id=frame_id, stamp=stamp)
            vis = make_split_view(left, right, total_w, total_h)

            if writer is not None:
                writer.write(vis)
            if jf is not None:
                jf.write(detections_to_json(rec) + "\n")
            if args.save_frames and processed % max(1, args.save_every) == 0:
                cv2.imwrite(os.path.join(args.save_frames, "split_%06d.jpg" % frame_id), vis,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])

            processed += 1
            if args.print_every and processed % args.print_every == 0:
                p = rec["pose"]
                print("  frame %5d  status=%-20s valid=%s  markers=%d  residual=%s  fps=%.1f" % (
                    frame_id, p.get("status"), p.get("valid"), p.get("marker_count", 0),
                    ("%.3f" % p["residual"]) if np.isfinite(p.get("residual", float("nan"))) else "---",
                    rec["fps"]))
            if not args.no_show:
                cv2.imshow("field localizer (q/ESC to quit)", vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            frame_id += 1
            if args.max_frames and processed >= args.max_frames:
                break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if jf is not None:
            jf.close()
        if not args.no_show:
            cv2.destroyAllWindows()

    elapsed = time.time() - t_start
    summary = pipe.summary()
    summary["wall_time_s"] = round(elapsed, 2)
    summary["source"] = str(args.source)
    summary["config"] = os.path.abspath(args.config)
    summary["field_yaml"] = pipe.field.source_path
    summary["field_yaml_sha256"] = pipe.field.sha256
    summary["intrinsics_rms_px"] = pipe.intr_cfg.get("rms")
    summary["intrinsics_views"] = pipe.intr_cfg.get("num_views")

    print("\n===== 汇总 =====")
    print("帧数 %d，平均 FPS %.1f（p50 %.1f），耗时 %.1fs" % (
        summary["frames"], summary["fps_mean"], summary["fps_p50"], elapsed))
    print("状态分布: %s" % json.dumps(summary["status_counts"], ensure_ascii=False))
    print("valid(ok) 占比: %.1f%%" % (100.0 * summary["valid_ratio"]))
    if summary["residual_mean_m"] is not None:
        print("残差: 均值 %.3f m / P95 %.3f m" % (summary["residual_mean_m"], summary["residual_p95_m"]))
    print("field.yaml sha256: %s" % (summary["field_yaml_sha256"] or "?"))

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            yaml.safe_dump(summary, f, allow_unicode=True, sort_keys=False)
        print("报告已写入 %s" % args.report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
