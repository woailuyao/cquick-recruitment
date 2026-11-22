"""Separate native AP metrics from fixed-threshold class-0 IoU=0.50 counts."""
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics
import time
from prepare_v3 import write_json, sha


def iou(a, b):
    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / union if union > 0 else 0.


def matches(gt, pred):
    used_g, used_p = set(), set()
    for overlap, i, j in sorted(((iou(g, p['box']), i, j) for i, g in enumerate(gt) for j, p in enumerate(pred)), reverse=True):
        if overlap >= .5 and i not in used_g and j not in used_p:
            used_g.add(i)
            used_p.add(j)
    return len(used_g), len(pred)-len(used_p), len(gt)-len(used_g), used_p


def fixed_summary(rows, saved, split):
    summaries, details = [], []
    for threshold in (.25, .50, .90, .95, .97, .99):
        counts = defaultdict(Counter)
        correct_conf = defaultdict(list)
        for r in rows:
            pred = [p for p in saved[r['output_name']] if p['cls'] == 0 and p['conf'] >= threshold]
            tp, fp, fn, used = matches(r['boxes'], pred)
            group = 'negative' if not r['boxes'] else 'backpack_occlusion' if 'occlusion_by_backpack' in r.get('quality_flags', []) else 'other_positive'
            for key in ('all', group):
                counts[key].update(images=1, gt=len(r['boxes']), predictions=len(pred), tp=tp, fp=fp, fn=fn)
                correct_conf[key].extend(pred[j]['conf'] for j in used)
            details.append(dict(split=split, threshold=threshold, image=r['output_name'], group=group,
                                gt=r['boxes'], predictions=pred, tp=tp, fp=fp, fn=fn))
        for group, counter in counts.items():
            d = dict(counter)
            tp, fp, fn = (d.get(k, 0) for k in ('tp', 'fp', 'fn'))
            conf = correct_conf[group]
            summaries.append(dict(split=split, threshold=threshold, group=group, **d,
                                  precision=tp/(tp+fp) if tp+fp else None,
                                  recall=tp/(tp+fn) if tp+fn else None,
                                  f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None,
                                  correct_conf_median=statistics.median(conf) if conf else None,
                                  correct_conf_max=max(conf) if conf else None))
    return summaries, details


def evaluate(weights, dataset, plan, split, output, device):
    from ultralytics import YOLO
    import torch
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    weights_hash = sha(weights)
    fingerprint = dict(weights_sha256=weights_hash, dataset_plan_id=sha(Path(__file__).parent/'plan_v3.json'), split=split)
    cache = output / 'summary.json'
    if cache.exists():
        old = json.loads(cache.read_text(encoding='utf-8'))
        if old['fingerprint'] != fingerprint:
            raise ValueError('评估缓存与模型或数据集不一致，拒绝覆盖：' + str(output))
        return old
    model = YOLO(str(weights))
    if model.names != {0: 'ball'}:
        raise ValueError('训练后模型类别不是0:ball。')
    # Low-threshold native validation for AP; separate from deployment threshold.
    m = model.val(data=str(dataset/'data.yaml'), split=split, imgsz=640, batch=1, device=device,
                  workers=2, conf=.001, iou=.7, max_det=300, plots=False, verbose=False,
                  project=str(output), name='native_metrics', exist_ok=True)
    native = dict(precision=float(m.box.mp), recall=float(m.box.mr), map50=float(m.box.map50), map50_95=float(m.box.map),
                  note='Ultralytics native AP evaluation at conf=.001. Summary P/R use its selected operating point; they are not fixed-.25 P/R.')
    rows = [r for r in plan['samples'] if r['split'] == split]
    params = dict(imgsz=640, conf=.25, iou=.7, max_det=20, device=device, verbose=False, save=False, rect=True)
    for _ in range(5):
        model.predict(source=str(dataset/'images'/split/rows[0]['output_name']), **params)
    sync = (lambda: torch.cuda.synchronize(int(device))) if device != 'cpu' else (lambda: None)
    sync()
    start, saved, speeds = time.perf_counter(), {}, []
    expected = {r['output_name'] for r in rows}
    for result in model.predict(source=str(dataset/'images'/split), stream=True, batch=1, **params):
        name = Path(result.path).name
        if name not in expected or name in saved:
            raise ValueError('预测图片名不一致或重复：' + name)
        saved[name] = [dict(box=b, conf=c, cls=int(k)) for b, c, k in zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist(), result.boxes.cls.cpu().tolist())]
        speeds.append(float(result.speed.get('inference', 0.)))
    sync()
    if set(saved) != expected:
        raise ValueError('评估未覆盖全部样本。')
    elapsed = time.perf_counter() - start
    fixed, details = fixed_summary(rows, saved, split)
    write_json(output/'details.json', details)
    result = dict(fingerprint=fingerprint, native=native, fixed_threshold_metrics=fixed,
                  timing=dict(mean_forward_inference_ms=statistics.mean(speeds), directory_seconds=elapsed,
                              directory_images_per_second=len(rows)/elapsed, note='Directory image evaluation, NOT camera FPS.'))
    write_json(cache, result)
    return result
