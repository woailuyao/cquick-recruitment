#!/usr/bin/env python3
"""Train n/s from COCO initialization, choose on val, then evaluate only the winner on test."""
import argparse
from datetime import datetime
import gc
import json
import platform
from pathlib import Path
import shutil
import zipfile
from prepare_v3 import BUNDLE, ROOT, prepare, sha, write_json
from evaluate_v3 import evaluate


def timestamp():
    return datetime.now().strftime('%Y%m%d_%H%M%S_%f')


def choose(val_results):
    return max(('n', 's'), key=lambda key: (val_results[key]['native']['map50_95'], key == 'n'))


def pretrained(variant, folder, YOLO):
    name = f'yolov8{variant}.pt'
    target = folder/name
    if not target.exists():
        for old in (ROOT/name, ROOT/'task1_ball'/name, Path.cwd()/name):
            if old.is_file() and old.resolve() != target.resolve():
                shutil.copy2(old, target)
                break
    if not target.exists():
        print('需要官方预训练权重：', name, '；尝试由Ultralytics官方下载。', flush=True)
        from ultralytics.utils.downloads import attempt_download_asset
        downloaded = Path(attempt_download_asset(str(target))).resolve()
        if not downloaded.is_file():
            raise ValueError('官方预训练权重未下载成功，请放到：' + str(target))
        if downloaded != target.resolve():
            shutil.copy2(downloaded, target)
    try:
        model = YOLO(str(target))
    except Exception as e:
        raise ValueError('预训练权重无法加载：' + str(target) + '；请重新取得官方同名文件。') from e
    if len(model.names) != 80 or model.names.get(0) != 'person':
        raise ValueError('需要官方80类COCO预训练权重，不能使用v1/v2或任何ball微调模型：' + str(target))
    del model
    return target


def train_one(variant, initial, state, state_path, config, dataset, experiment, YOLO):
    record = state.setdefault('models', {}).get(variant, {})
    if record and record['initial_sha256'] != sha(initial):
        raise ValueError('预训练文件已改变，拒绝继续混用实验。')
    previous = Path(record.get('run_dir', '/nonexistent'))
    best, last = previous/'weights/best.pt', previous/'weights/last.pt'
    if record.get('status') == 'trained':
        if not best.is_file() or sha(best) != record['best_sha256']:
            raise ValueError('已完成模型被修改/缺失：' + str(best))
        print('复用本次已完成的v3模型：', best, flush=True)
        return best
    if record.get('status') == 'training' and last.is_file():
        model = YOLO(str(last))
        if model.ckpt and model.ckpt.get('epoch', -1) == -1 and best.is_file():
            pass  # Ultralytics finalized training before our state file was saved.
        else:
            print('继续中断的v3训练：', last, flush=True)
            model.train(resume=True)
            previous = Path(model.trainer.save_dir)
            best = previous/'weights/best.pt'
    else:
        previous = experiment / f'ball_yolov8{variant}_{timestamp()}'
        record = dict(status='training', run_dir=str(previous), initial_weights=str(initial), initial_sha256=sha(initial))
        state['models'][variant] = record
        write_json(state_path, state)
        model = YOLO(str(initial))
        if len(model.names) != 80:
            raise ValueError('拒绝从已训练足球模型开始正式实验。')
        print('\n开始从官方COCO权重训练YOLOv8' + variant, flush=True)
        model.train(data=str(dataset/'data.yaml'), project=str(experiment), name=previous.name, exist_ok=False,
                    device=config['device'], **config['training'])
        previous = Path(model.trainer.save_dir)
        best = previous/'weights/best.pt'
    if not best.is_file():
        raise ValueError('训练结束但找不到best.pt：' + str(best))
    record.update(status='trained', run_dir=str(previous), best_sha256=sha(best))
    state['models'][variant] = record
    write_json(state_path, state)
    return best


def save_archive(dataset, experiment, plan, state, val_results, winner, test_result):
    report = dict(scope=plan['scope'], source_round_note=plan['source_round_note'],
                  split_counts=dict(train_original=451, train_total=514, val=145, test=76),
                  selection_rule=plan['selection_rule'], chosen_variant=winner,
                  chosen_best=str(Path(state['models'][winner]['run_dir'])/'weights/best.pt'),
                  validation=val_results, final_test=test_result,
                  limitations=['Validation has no empty frames; final test has only 2 empty frames.',
                               'No wholly independent held-out backpack video. Training-side backpack replay is not a test.',
                               'Video files isolated, but laboratory/session shared and all material previously observed in development.',
                               'Do not tune hyperparameters or choose another model after reading final test results.'])
    write_json(experiment/'report.json', report)
    lines = ['# 第三版训练与评估结果', '', '模型按验证集mAP50-95选择，测试集未参与模型选择。',
             '', f"最终模型：YOLOv8{winner}", f"权重：{report['chosen_best']}", '',
             '| 模型 | 验证P | 验证R | 验证mAP50 | 验证mAP50-95 |', '|---|---:|---:|---:|---:|']
    for key in ('n', 's'):
        m = val_results[key]['native']
        lines.append(f"| YOLOv8{key} | {m['precision']:.4f} | {m['recall']:.4f} | {m['map50']:.4f} | {m['map50_95']:.4f} |")
    m = test_result['native']
    lines.extend(['', '## 最终测试集（只评估验证集选出的模型）', '',
                  f"P={m['precision']:.4f}，R={m['recall']:.4f}，mAP50={m['map50']:.4f}，mAP50-95={m['map50_95']:.4f}。",
                  '', '上述P/R来自Ultralytics原生评估的工作点；固定0.25/0.97等阈值结果见report.json，不与置信度混为一谈。',
                  '', '测试按完整视频隔离，但素材此前已用于开发观察；不代表全新场景盲测。验证无空帧、测试只有2张空帧，误检评估覆盖有限。',
                  '本实验没有独立的背包遮挡测试视频，不能据此声称遮挡泛化能力改善。',
                  '相机实时检测、FPS实测、录屏、B站链接、GitHub提交及现场演示仍需另行完成。'])
    (experiment/'RESULTS.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    archive = dataset/'v3_results_for_review.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in ('report.json', 'RESULTS.md', 'selection_before_test.json', 'experiment_state.json'):
            z.write(experiment/name, name)
        z.write(dataset/'selection_v3.csv', 'selection_v3.csv')
        for f in sorted((experiment/'evaluation').rglob('*.json')):
            z.write(f, str(f.relative_to(experiment)))
        for key, record in state['models'].items():
            for name in ('results.csv', 'results.png', 'args.yaml'):
                f = Path(record['run_dir'])/name
                if f.is_file():
                    z.write(f, f'training_{key}/{name}')
    print('\n选定模型：', report['chosen_best'], flush=True)
    print('请上传这个小结果包：', archive, flush=True)
    return archive


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', default=str(ROOT/'task1_ball/opencv_expansion_v1'))
    p.add_argument('--dataset', default=str(ROOT/'task1_ball/yolo_ball_v3'))
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--batch', type=int, default=4)
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--device', default='0')
    p.add_argument('--new-run', action='store_true', help='仅在最终测试尚未开始前创建新实验；旧结果保留')
    a = p.parse_args()
    if a.batch < 1 or a.epochs < 1 or (a.device != 'cpu' and not a.device.isdigit()):
        raise ValueError('batch/epochs需为正数，device为GPU编号或cpu。')
    dataset, plan = prepare(a.project, a.dataset)
    if a.prepare_only:
        return
    import torch
    import ultralytics
    from ultralytics import YOLO
    if a.device != 'cpu' and not torch.cuda.is_available():
        raise ValueError('CUDA不可用，请使用Week3-4/.venv。')
    environment = dict(python=platform.python_version(), torch=getattr(torch, '__version__', 'unknown'),
                       ultralytics=getattr(ultralytics, '__version__', 'unknown'),
                       device=a.device, gpu=torch.cuda.get_device_name(int(a.device)) if a.device != 'cpu' else None)
    print('训练环境：', environment, flush=True)
    from matplotlib import font_manager
    from PIL import ImageFont
    import ultralytics.utils.checks as checks
    font = Path(checks.USER_CONFIG_DIR)/'Arial.ttf'
    try:
        ImageFont.truetype(str(font), 16)
    except OSError:
        font.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(font_manager.findfont('DejaVu Sans'), font)
    config = dict(plan_sha256=sha(BUNDLE/'plan_v3.json'), device=a.device, training=dict(
        epochs=a.epochs, patience=20, imgsz=640, batch=a.batch, workers=2, cache=False, amp=True,
        optimizer='AdamW', lr0=.001, lrf=.01, cos_lr=True, warmup_epochs=3, weight_decay=.0005,
        mosaic=.5, close_mosaic=10, scale=.3, translate=.1, fliplr=.5, flipud=0., hsv_s=.4, hsv_v=.3,
        mixup=0., copy_paste=0., seed=42, deterministic=True, plots=True, val=True, split='val', resume=False))
    pointer = dataset/'v3_experiment.json'
    state = json.loads(pointer.read_text(encoding='utf-8')) if pointer.exists() else {}
    if a.new_run and state.get('test_started'):
        raise ValueError('最终测试已开始/完成，不能继续用同一test反复调参。保留现有结果并补充新的测试素材。')
    if state and state['config'] != config and not a.new_run:
        raise ValueError('参数与未完成实验不同；若需降batch且最终测试尚未开始，添加 --new-run 保留旧结果并新建实验。')
    if not state or a.new_run:
        experiment = ROOT/'results/task1_ball_yolo'/('formal_v3_'+timestamp())
        experiment.mkdir(parents=True, exist_ok=False)
        state = dict(config=config, environment=environment, experiment=str(experiment), models={}, created=datetime.now().isoformat(),
                     test_started=False, test_completed=False)
        write_json(pointer, state)
    experiment = Path(state['experiment'])
    state_path = experiment/'experiment_state.json'
    # The pointer carries configuration and state only; model paths remain separate from v1/v2.
    if state_path.exists():
        state = json.loads(state_path.read_text(encoding='utf-8'))
        if state['config'] != config:
            raise ValueError('实验状态参数不一致。')
    if a.new_run and state.get('test_started'):
        raise ValueError('最终测试已经开始，禁止重建。')
    if state.get('test_completed'):
        for record in state['models'].values():
            if sha(Path(record['run_dir'])/'weights/best.pt') != record['best_sha256']:
                raise ValueError('已完成实验的模型被修改。')
        validation = {k: json.loads((experiment/'evaluation'/('val_'+k)/'summary.json').read_text(encoding='utf-8')) for k in ('n', 's')}
        final = json.loads((experiment/'evaluation'/'final_test'/'summary.json').read_text(encoding='utf-8'))
        frozen = json.loads((experiment/'selection_before_test.json').read_text(encoding='utf-8'))
        save_archive(dataset, experiment, plan, state, validation, frozen['variant'], final)
        print('实验已经完成，以上仅重新打包，没有重新训练或重测。', flush=True)
        return
    write_json(state_path, state)
    pretrained_dir = ROOT/'pretrained_coco'
    pretrained_dir.mkdir(exist_ok=True)
    # Fetch both initializations before committing GPU time to training.
    initial = {key: pretrained(key, pretrained_dir, YOLO) for key in ('n', 's')}
    best, validation = {}, {}
    for key in ('n', 's'):
        best[key] = train_one(key, initial[key], state, state_path, config, dataset, experiment, YOLO)
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        validation[key] = evaluate(best[key], dataset, plan, 'val', experiment/'evaluation'/('val_'+key), a.device)
    winner = choose(validation)
    frozen = dict(variant=winner, best=str(best[winner]), best_sha256=sha(best[winner]),
                  rule=plan['selection_rule'], validation_map50_95={k: v['native']['map50_95'] for k, v in validation.items()},
                  plan_sha256=config['plan_sha256'])
    freeze_path = experiment/'selection_before_test.json'
    if freeze_path.exists() and json.loads(freeze_path.read_text(encoding='utf-8')) != frozen:
        raise ValueError('模型选择与最终测试前的冻结记录不一致，停止。')
    write_json(freeze_path, frozen)
    state['test_started'] = True
    write_json(state_path, state)
    write_json(pointer, state)
    print('\n验证集选出YOLOv8' + winner + '，选择已冻结。现在只评估该模型的测试集。', flush=True)
    final = evaluate(best[winner], dataset, plan, 'test', experiment/'evaluation'/'final_test', a.device)
    state['test_completed'] = True
    write_json(state_path, state)
    write_json(pointer, state)
    save_archive(dataset, experiment, plan, state, validation, winner, final)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as e:
        raise SystemExit(str(e))
