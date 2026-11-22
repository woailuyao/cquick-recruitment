"""任务 2：参数对比实验。

同一张图、只改一个参数，跑两种配置，拼成一张对比图并打印对比表。
覆盖考核要求的两种对比：
    A) HSV 上下界变化
    B) 最小轮廓面积变化

用法：
    python3 -m color_target_detector.param_compare --image data/test1.png --case hsv
    python3 -m color_target_detector.param_compare --image data/test1.png --case area
"""

import argparse
import os
import sys

import cv2
import numpy as np

from .detection import DEFAULT_LOWER, DEFAULT_MIN_AREA, DEFAULT_UPPER, detect, draw_result

CASES = {
    "hsv": dict(
        name="A: HSV 区间变化",
        a=dict(lower=(35, 80, 60), upper=(85, 255, 255), min_area=500, morph="open"),
        b=dict(lower=(52, 145, 100), upper=(85, 255, 255), min_area=500, morph="open"),
        desc="只收紧下界：H 35→52、S 80→145、V 60→100，把「偏灰的绿草地」排除掉",
    ),
    "area": dict(
        name="B: 最小面积变化",
        a=dict(lower=(35, 80, 60), upper=(85, 255, 255), min_area=500, morph="open"),
        b=dict(lower=(35, 80, 60), upper=(85, 255, 255), min_area=8000, morph="open"),
        desc="只提高面积阈值：500 → 8000，小干扰目标会被过滤",
    ),
    "morph": dict(
        name="C: 形态学开运算对比",
        # 注意：这里把 min_area 降到 8，才能看出开运算对“小噪点”的作用；
        # min_area=500 时噪点本来就被面积阈值挡掉了，开运算看不出差别。
        a=dict(lower=(35, 80, 60), upper=(85, 255, 255), min_area=8, morph="none"),
        b=dict(lower=(35, 80, 60), upper=(85, 255, 255), min_area=8, morph="open"),
        desc="只改形态学：none vs open（先把面积阈值降到 8，噪点才会成为候选）",
    ),
}


def imread_utf8(path):
    """读图：兼容含中文/生僻字的路径（cv2.imread 对这种路径会失败）。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_utf8(path, img):
    """写图：兼容含中文/生僻字的路径。"""
    ext = os.path.splitext(path)[1]
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(path)


def run_case(image, params):
    return detect(image, params["lower"], params["upper"],
                  params["min_area"], params["morph"])


def label(img, text, color=(255, 255, 0)):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="HSV 参数对比实验")
    p.add_argument("--image", required=True)
    p.add_argument("--case", default="hsv", choices=list(CASES))
    p.add_argument("--out", default="")
    args = p.parse_args(argv)

    image = imread_utf8(args.image)
    if image is None:
        print(f"[ERROR] 图片读取失败: {args.image}")
        return 1

    case = CASES[args.case]
    det_a = run_case(image, case["a"])
    det_b = run_case(image, case["b"])

    print(f"=== {case['name']} ===")
    print(f"说明: {case['desc']}")
    print(f"图片: {args.image}  ({image.shape[1]}x{image.shape[0]})")
    print()
    for tag, params, det in (("A", case["a"], det_a), ("B", case["b"], det_b)):
        print(f"[配置 {tag}] lower={params['lower']}  upper={params['upper']}  "
              f"min_area={int(params['min_area'])}  morph={params['morph']}")
        if det.detected:
            print(f"         → 检测到   u={det.u:7.1f}  v={det.v:7.1f}  "
                  f"area={det.area:9.0f}  候选数={det.n_candidates}")
        else:
            print(f"         → 未检测到 （通过面积阈值的轮廓数=0）")

    # 差异分析（写 README 时可以直接抄这一段结论）
    print()
    if det_a.detected and det_b.detected:
        du = det_b.u - det_a.u
        dv = det_b.v - det_a.v
        ratio = det_b.area / det_a.area if det_a.area > 0 else 0.0
        print(f"[差异] area {det_a.area:.0f} → {det_b.area:.0f}（×{ratio:.2f}）；"
              f"中心点偏移 du={du:+.1f}px dv={dv:+.1f}px")
        if ratio > 2.0 or ratio < 0.5:
            print("[结论] 面积变化超过 2 倍 —— 被检区域发生了本质变化：")
            print("       多半是某个配置把『背景/大片同色区域』也算成了目标（误检），")
            print("       或把目标的一部分切掉了（轮廓不完整）。")
        elif abs(du) > 5 or abs(dv) > 5:
            print("[结论] 面积接近但中心点漂移明显 —— 轮廓形状被改变，质心跟着移动。")
        else:
            print("[结论] 面积与中心点都基本稳定 —— 该参数对定位结果影响很小。")
        if det_a.n_candidates != det_b.n_candidates:
            print(f"[结论] 候选数量 {det_a.n_candidates} → {det_b.n_candidates}，"
                  f"说明该参数在影响『有几个东西被当成目标』。")
    elif det_a.detected and not det_b.detected:
        print("[结论] A 检测到、B 没检测到 → B 的参数过严，属于【漏检】。")
    elif not det_a.detected and det_b.detected:
        print("[结论] A 没检测到、B 检测到 → A 的参数过严，属于【漏检】。")
    else:
        print("[结论] 两种配置都没检测到 → 目标颜色/光照与 HSV 阈值不匹配，")
        print("       或 min_area 设得比目标本身还大。")

    # 拼对比图
    img_a = label(draw_result(image, det_a), f"A: {case['a']['lower']}~{case['a']['upper']} minArea={int(case['a']['min_area'])} {case['a']['morph']}")
    img_b = label(draw_result(image, det_b), f"B: {case['b']['lower']}~{case['b']['upper']} minArea={int(case['b']['min_area'])} {case['b']['morph']}")
    gap = np.full((image.shape[0], 8, 3), 255, np.uint8)
    compare = np.hstack([img_a, gap, img_b])

    out_path = args.out or f"out/compare_{args.case}_{os.path.splitext(os.path.basename(args.image))[0]}.png"
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    imwrite_utf8(out_path, compare)
    print(f"[INFO] 对比图已保存: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
