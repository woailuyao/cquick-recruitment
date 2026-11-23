#!/usr/bin/env python3
"""Camera detection with pixel coordinates, timing and explicit replay identification."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

ROOT = Path.home() / 'Cquick/Week3-4'
BEST = ROOT / 'results/task1_ball_yolo/ball_yolov8n_v2_20261114_201006_173573/weights/best.pt'


def camera_source(value):
    return value.isdigit() or value.startswith('/dev/video')


def detections(result):
    return [dict(class_id=0, name='ball', confidence=float(c), x1=float(b[0]), y1=float(b[1]),
                 x2=float(b[2]), y2=float(b[3]), center_u=float((b[0] + b[2]) / 2),
                 center_v=float((b[1] + b[3]) / 2))
            for b, c, k in zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist(),
                               result.boxes.cls.cpu().tolist()) if int(k) == 0]


def probe(cap, cv2, count):
    for _ in range(15):
        ok, frame = cap.read()
        if not ok:
            raise ValueError('相机预热取帧失败，请检查设备连接和占用。')
    start, stamps = time.perf_counter(), []
    for _ in range(count):
        ok, frame = cap.read()
        if not ok:
            raise ValueError('相机连续取帧失败。')
        stamps.append(time.perf_counter())
    height, width = frame.shape[:2]
    return dict(width=width, height=height, warmup_frames=15, sample_frames=count,
                received_fps=(count - 1) / (stamps[-1] - stamps[0]),
                elapsed_seconds=stamps[-1] - start, device_reported_fps=cap.get(cv2.CAP_PROP_FPS),
                method='Consecutive cap.read, no inference/display; after 15 warmup frames. Received rate, not sensor exposure timestamps.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', default='/dev/video0')
    p.add_argument('--weights', default=str(BEST))
    p.add_argument('--device', default='0')
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--conf', type=float, default=.25)
    p.add_argument('--probe-only', action='store_true')
    p.add_argument('--probe-frames', type=int, default=120)
    p.add_argument('--max-frames', type=int, default=0, help='0表示持续运行；按q或Ctrl+C结束')
    p.add_argument('--no-display', action='store_true')
    a = p.parse_args()
    if not 0 < a.conf <= 1 or a.imgsz <= 0 or a.imgsz % 32 or a.probe_frames < 2 or a.max_frames < 0:
        raise ValueError('参数无效：conf在(0,1]内，imgsz为正32倍数，probe-frames>=2。')
    is_camera = camera_source(a.source)
    if a.probe_only and not is_camera:
        raise ValueError('相机帧率自检只能对真实相机运行。')
    if not is_camera and not Path(a.source).expanduser().is_file():
        raise ValueError('找不到回放视频：' + a.source)
    weights = Path(a.weights).expanduser().resolve()
    if not a.probe_only and not weights.is_file():
        raise ValueError('找不到best.pt：' + str(weights))
    import cv2
    source = int(a.source) if a.source.isdigit() else str(Path(a.source).expanduser())
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2) if is_camera else cv2.VideoCapture(source)
    if not cap.isOpened():
        cap.release()
        raise ValueError('无法打开输入。WSL中请先用 v4l2-ctl --list-devices 确认设备已接入；关闭占用相机的软件。')
    if is_camera:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    mode = 'LIVE CAMERA' if is_camera else 'PRERECORDED REPLAY - NOT LIVE'
    out = ROOT / 'results/task1_ball_checks' / datetime.now().strftime('camera_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    meta = dict(source=a.source, mode=mode, start_time=datetime.now().astimezone().isoformat(),
                operator='hu_yuhan', frame_channels='BGR', imgsz=a.imgsz, threshold=a.conf,
                output_fps_definition='Measured processed frames / wall time, includes read, predict, display and JSON logging; excludes startup/warmup.',
                inference_ms_definition='Ultralytics reported forward inference time; excludes preprocessing/postprocessing.',
                capture_probe=None, frames=0, error=None)
    window_opened = False
    try:
        print('姓名: hu_yuhan |', meta['start_time'], '|', mode, flush=True)
        if is_camera:
            meta['capture_probe'] = probe(cap, cv2, a.probe_frames)
            print('相机自检：', meta['capture_probe'], flush=True)
            if (meta['capture_probe']['width'], meta['capture_probe']['height']) != (1280, 720):
                raise ValueError('相机未输出1280x720；请先检查支持格式或取得组长指定的替代分辨率。')
        if a.probe_only:
            return
        import torch
        from ultralytics import YOLO
        if a.device != 'cpu' and not torch.cuda.is_available():
            raise ValueError('CUDA不可用，请使用Week3-4/.venv，或指定 --device cpu。')
        model = YOLO(str(weights))
        if model.names != {0: 'ball'}:
            raise ValueError('模型类别不符合要求，应仅有 0: ball；实际：' + str(model.names))
        meta['weights'] = str(weights)
        meta['weights_sha256'] = hashlib.sha256(weights.read_bytes()).hexdigest()
        meta['versions'] = dict(opencv=cv2.__version__, torch=torch.__version__)
        ok, warm = cap.read()
        if not ok:
            raise ValueError('启动检测前取帧失败。')
        kwargs = dict(imgsz=a.imgsz, conf=a.conf, iou=.7, max_det=20, device=a.device,
                      save=False, verbose=False, rect=True)
        for _ in range(5):
            model.predict(source=warm, **kwargs)
        if a.device != 'cpu':
            torch.cuda.synchronize(int(a.device))
        start, count, total_predict, total_infer = time.perf_counter(), 0, 0., 0.
        with (out / 'detections.jsonl').open('w', encoding='utf-8') as log:
            while not a.max_frames or count < a.max_frames:
                ok, frame = cap.read()
                if not ok:
                    if is_camera:
                        raise ValueError('相机中途断开或取帧失败。')
                    break
                received = datetime.now().astimezone().isoformat(timespec='milliseconds')
                before = time.perf_counter()
                result = model.predict(source=frame, **kwargs)[0]
                boxes = detections(result)
                predict_ms = (time.perf_counter() - before) * 1000
                count += 1
                total_predict += predict_ms
                infer_ms = float(result.speed.get('inference', 0.))
                total_infer += infer_ms
                output_fps = count / (time.perf_counter() - start)
                record = dict(frame_id=count, received_time=received, mode=mode, detections=boxes,
                              predict_call_ms=predict_ms, inference_ms=infer_ms, output_fps=output_fps)
                log.write(json.dumps(record, ensure_ascii=False) + '\n')
                if count % 30 == 0:
                    log.flush()
                    print(json.dumps(record, ensure_ascii=False), flush=True)
                quit_requested = False
                if not a.no_display:
                    for d in boxes:
                        x1, y1, x2, y2 = (int(d[k]) for k in ('x1', 'y1', 'x2', 'y2'))
                        u, v = int(d['center_u']), int(d['center_v'])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 80, 0), 2)
                        cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                        cv2.putText(frame, f"ball {d['confidence']:.3f} center=({u},{v})", (x1, max(65, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 80, 0), 2)
                    cv2.putText(frame, f'{mode} | hu_yuhan | frame {count}', (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
                    cv2.putText(frame, f'{received} | FPS {output_fps:.1f} | infer {infer_ms:.1f}ms', (12, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                    cv2.imshow('CQuick ball detection - q to quit', frame)
                    window_opened = True
                    quit_requested = cv2.waitKey(1) & 0xff == ord('q')
                meta.update(frames=count, processed_fps=count / (time.perf_counter() - start),
                            mean_predict_call_ms=total_predict / count, mean_inference_ms=total_infer / count)
                if quit_requested:
                    break
    except KeyboardInterrupt:
        meta['stop_reason'] = 'keyboard_interrupt'
    except Exception as e:
        meta['error'] = str(e)
        raise
    finally:
        cap.release()
        if window_opened:
            cv2.destroyAllWindows()
        meta['end_time'] = datetime.now().astimezone().isoformat()
        (out / 'session_summary.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
        print('本次记录：', out, flush=True)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as e:
        raise SystemExit(str(e))
