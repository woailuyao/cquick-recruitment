#!/usr/bin/env python3
"""Build a separate train/val/test dataset from the reviewed original project."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

BUNDLE = Path(__file__).resolve().parent
SPLITS = ('train', 'val', 'test')
ROOT = Path.home() / 'Cquick/Week3-4'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def write_json(path, data):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    tmp.replace(path)


def yaml_text(output):
    return ('path: ' + json.dumps(str(output), ensure_ascii=False) +
            '\ntrain: images/train\nval: images/val\ntest: images/test\nnc: 1\nnames:\n  0: ball\n')


def plan_checks(plan):
    rows = plan['samples']
    if len({r['output_name'] for r in rows}) != len(rows):
        raise ValueError('输出文件名重复。')
    if Counter(r['split'] for r in rows) != Counter(train=514, val=145, test=76):
        raise ValueError('v3样本数与冻结方案不一致。')
    owners = {}
    for r in rows:
        if r['split'] not in SPLITS or Path(r['output_name']).name != r['output_name'] or Path(r['parent_image']).name != r['parent_image']:
            raise ValueError('集合或文件名无效。')
        if r['split'] != plan['group_assignment'][r['source_group']]:
            raise ValueError('同一视频跨集合：' + r['source_group'])
        for prefix, key in [('parent', r['parent_image']), ('bytes', r['jpeg_sha256'])]:
            old = owners.setdefault((prefix, key), r['split'])
            if old != r['split']:
                raise ValueError('同一原图/完全相同图片跨集合：' + key)
        if r['kind'] != 'original' and r['split'] != 'train':
            raise ValueError('派生训练样本进入验证/测试。')
        for x1, y1, x2, y2 in r['boxes']:
            if not 0 <= x1 < x2 <= r['width'] or not 0 <= y1 < y2 <= r['height']:
                raise ValueError('标注框越界：' + r['output_name'])
    originals = {r['image'] for r in rows if r['kind'] == 'original' and r['split'] == 'train'}
    if any(r['parent_image'] not in originals for r in rows if r['kind'] != 'original'):
        raise ValueError('派生样本的原图未属于训练集合。')


def verify_output(output, plan):
    if (output / 'dataset_plan_id.txt').read_text().strip() != sha(BUNDLE / 'plan_v3.json'):
        raise ValueError('现有输出目录属于不同划分，拒绝覆盖。')
    for split in SPLITS:
        rows = [r for r in plan['samples'] if r['split'] == split]
        expected_images = {r['output_name'] for r in rows}
        expected_labels = {Path(r['output_name']).stem + '.txt' for r in rows}
        if {p.name for p in (output / 'images' / split).iterdir()} != expected_images:
            raise ValueError('图片集合不一致：' + split)
        if {p.name for p in (output / 'labels' / split).iterdir()} != expected_labels:
            raise ValueError('标签集合不一致：' + split)
        for r in rows:
            if sha(output / 'images' / split / r['output_name']) != r['jpeg_sha256']:
                raise ValueError('图片被修改：' + r['output_name'])
            label = output / 'labels' / split / (Path(r['output_name']).stem + '.txt')
            if label.read_text(encoding='utf-8') != r['label_text']:
                raise ValueError('标签被修改：' + r['output_name'])
    if (output / 'data.yaml').read_text(encoding='utf-8') != yaml_text(output):
        raise ValueError('data.yaml不一致。')


def prepare(project, output):
    project, output = Path(project).expanduser().resolve(), Path(output).expanduser().resolve()
    if output == project or project in output.parents or output in project.parents:
        raise ValueError('新数据目录与原复核项目必须互不包含。')
    plan = json.loads((BUNDLE / 'plan_v3.json').read_text(encoding='utf-8'))
    plan_checks(plan)
    refs = json.loads((BUNDLE / 'review_reference.json').read_text(encoding='utf-8'))
    state = json.loads((project / 'review_state.json').read_text(encoding='utf-8'))
    current = {r['image']: r for r in state}
    if len(state) != 768 or len(current) != 768 or set(current) != {r['image'] for r in refs}:
        raise ValueError('原复核状态与768张已交付记录不一致。')
    for ref in refs:
        for k in ('status', 'boxes', 'source_group', 'exact_decoded_sha256', 'width', 'height'):
            if current[ref['image']].get(k) != ref[k]:
                raise ValueError('复核记录不一致：' + ref['image'] + ' ' + k)
        label = project / 'reviewed_labels' / (Path(ref['image']).stem + '.txt')
        if ref['status'] == 'accepted':
            if not label.is_file() or label.read_text(encoding='utf-8') != ref['label_text']:
                raise ValueError('原复核标签不一致：' + ref['image'])
        elif label.exists():
            raise ValueError('剔除图仍有标签：' + ref['image'])
    sources, checked = {}, {}
    for r in plan['samples']:
        source = (BUNDLE / r['asset']).resolve() if 'asset' in r else project / 'images' / r['parent_image']
        if 'asset' in r and not source.is_relative_to(BUNDLE):
            raise ValueError('裁剪资源路径越界。')
        if source not in checked:
            checked[source] = sha(source)
        if checked[source] != r['jpeg_sha256']:
            raise ValueError('原图片/资源哈希不一致：' + r['output_name'])
        sources[r['output_name']] = source
    if output.exists():
        verify_output(output, plan)
        print('v3数据集已存在，完整核验通过。', flush=True)
        return output, plan
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.' + output.name + '_staging_', dir=output.parent))
    try:
        for split in SPLITS:
            (stage / 'images' / split).mkdir(parents=True)
            (stage / 'labels' / split).mkdir(parents=True)
        for r in plan['samples']:
            shutil.copy2(sources[r['output_name']], stage / 'images' / r['split'] / r['output_name'])
            label = stage / 'labels' / r['split'] / (Path(r['output_name']).stem + '.txt')
            label.write_text(r['label_text'], encoding='utf-8')
        (stage / 'dataset_plan_id.txt').write_text(sha(BUNDLE / 'plan_v3.json'), encoding='utf-8')
        (stage / 'data.yaml').write_text(yaml_text(output), encoding='utf-8')
        for name in ('selection_v3.csv', 'README.md'):
            shutil.copy2(BUNDLE / name, stage / name)
        # Verify every staged image before publishing the directory.
        for r in plan['samples']:
            if sha(stage / 'images' / r['split'] / r['output_name']) != r['jpeg_sha256']:
                raise ValueError('复制图片校验失败。')
        stage.rename(output)
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    verify_output(output, plan)
    print('v3生成完成：训练514份（451张原图+63份派生/重复），验证145张，测试76张。', flush=True)
    return output, plan


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', default=str(ROOT / 'task1_ball/opencv_expansion_v1'))
    p.add_argument('--output', default=str(ROOT / 'task1_ball/yolo_ball_v3'))
    a = p.parse_args()
    try:
        prepare(a.project, a.output)
    except (ValueError, OSError, KeyError) as e:
        raise SystemExit(str(e))
