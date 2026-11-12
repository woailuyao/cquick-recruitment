# -*- coding: utf-8 -*-
"""
任务一：图像基础操作

对 data/images 下每张图做缩放、裁剪、灰度转换，结果存到 outputs/task1，
写完再读回来校验；原图只读不覆盖。

用法：python src/task1_basics.py
"""

import hashlib
import sys
from pathlib import Path

import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task1"

FIXED_W = 512                 # 固定宽度缩放的目标宽度
CROP_CENTER = (480, 360)      # 中心裁剪的 (w, h)
CROP_RECT = (640, 200, 480, 360)   # 任意区域裁剪 (x, y, w, h)


# --------------------------------------------------------------------------- #
# 中文路径安全的读写
# --------------------------------------------------------------------------- #
def imread_u(path, flags=cv2.IMREAD_COLOR):
    """读图；文件不存在/为空/损坏时返回 None（不抛异常）。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return None if data.size == 0 else cv2.imdecode(data, flags)


def imwrite_u(path, img):
    ok, buf = cv2.imencode(Path(path).suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------------------- #
# 单张图片的处理
# --------------------------------------------------------------------------- #
def process_one(path, log):
    """
    返回 True 表示全部步骤成功。
    日志同时写控制台和 run_log.txt。
    """
    def say(msg=""):
        print(msg)
        log.append(msg)

    say("=" * 78)
    say(f"[文件] {path.name}")
    say(f"[路径] {path.relative_to(ROOT)}")

    # ---------- 步骤 1：读取并检查 ----------
    img = imread_u(path)
    if img is None:
        say("[FAIL] 读取失败：文件不存在、扩展名错误或图像已损坏")
        return False

    h, w = img.shape[:2]
    c = img.shape[2] if img.ndim == 3 else 1
    size_kb = path.stat().st_size / 1024
    say(f"[读取] 成功  shape={img.shape}  dtype={img.dtype}  "
        f"高={h} 宽={w} 通道={c}  文件={size_kb:.1f} KB")
    # OpenCV 的 shape 是 (高, 宽, 通道)，与习惯上的"宽 x 高"顺序相反，这里特意分开打印
    say(f"       注意：shape[0]=高={h}，shape[1]=宽={w}；"
        f"矩阵索引是 img[y, x]，不是 img[x, y]")

    stem = path.stem
    outputs = []          # [(文件名, 写入时用的数组)]

    def save(name, arr, note):
        p = OUT_DIR / name
        imwrite_u(p, arr)
        outputs.append((name, arr))
        say(f"{note}  ({name})")
        return p

    # ---------- 步骤 2a：按比例缩放 0.5 ----------
    small = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    save(f"{stem}_resize_scale0.5.png", small,
         f"[缩放] 比例 0.5      -> {small.shape[1]}x{small.shape[0]}")

    # ---------- 步骤 2b：按固定宽度缩放（保持宽高比） ----------
    ratio = FIXED_W / w
    fixed = cv2.resize(img, (FIXED_W, int(round(h * ratio))), interpolation=cv2.INTER_AREA)
    save(f"{stem}_resize_w{FIXED_W}.png", fixed,
         f"[缩放] 固定宽度 {FIXED_W} -> {fixed.shape[1]}x{fixed.shape[0]}  缩放比={ratio:.3f}")

    # ---------- 步骤 2c：中心裁剪 ----------
    cw, ch = CROP_CENTER
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    center_crop = img[y0:y0 + ch, x0:x0 + cw].copy()
    save(f"{stem}_crop_center.png", center_crop,
         f"[裁剪] 中心区域     -> x={x0} y={y0} w={cw} h={ch}  结果 {center_crop.shape[1]}x{center_crop.shape[0]}")

    # ---------- 步骤 2d：指定区域裁剪 ----------
    rx, ry, rw, rh = CROP_RECT
    rx, ry = min(rx, w - rw), min(ry, h - rh)
    region = img[ry:ry + rh, rx:rx + rw].copy()
    save(f"{stem}_crop_region.png", region,
         f"[裁剪] 指定区域     -> x={rx} y={ry} w={rw} h={rh}")

    # ---------- 步骤 2e：灰度转换 ----------
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    save(f"{stem}_gray.png", gray,
         f"[灰度] BGR2GRAY     -> shape={gray.shape} 通道=1  灰度范围=[{gray.min()}, {gray.max()}]")

    # ---------- 步骤 2f：BGR 与 RGB 的区别（演示用） ----------
    b, g, r = cv2.split(img)
    swapped = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)      # 被当成 RGB 送去显示/保存的错误顺序
    demo = cv2.hconcat([img, swapped])
    save(f"{stem}_bgr_vs_rgb.png", demo, f"[通道] 左=原始 BGR 读入，右=通道互换后")
    bx, by = w // 2, h // 2
    say(f"       像素({bx},{by})  B={img[by, bx, 0]} G={img[by, bx, 1]} R={img[by, bx, 2]}")
    say(f"       cv2.split 三通道均值：B={b.mean():.1f} G={g.mean():.1f} R={r.mean():.1f}")
    say("       OpenCV 默认按 BGR 存图；若把它当 RGB 直接给 matplotlib/PIL 显示，"
        "红蓝会互换（见 *_bgr_vs_rgb.png 右半部分）")

    # ---------- 步骤 3：写入后重新读取校验 ----------
    all_ok = True
    for name, arr in outputs:
        fp = OUT_DIR / name
        if not fp.exists():
            say(f"[FAIL] 文件未生成：{name}")
            all_ok = False
            continue
        # 单通道灰度图要用 IMREAD_UNCHANGED 读回，否则默认 IMREAD_COLOR 会扩成 3 通道
        back = imread_u(fp, cv2.IMREAD_UNCHANGED)
        if back is None:
            say(f"[FAIL] 无法重新读取：{name}")
            all_ok = False
            continue
        same = back.shape == arr.shape and np.array_equal(back, arr)
        say(f"[校验] {name}  重读 shape={back.shape}  "
            f"{'与内存一致' if same else '与内存不一致！'}  size={fp.stat().st_size / 1024:.1f} KB")
        all_ok = all_ok and same

    say(f"[结果] {len(outputs)} 个输出文件，全部可重新读取：{all_ok}")
    return all_ok


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    files = sorted([p for p in IMG_DIR.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}])
    if not files:
        print(f"[错误] {IMG_DIR} 下没有图片，请先运行 python src/make_test_images.py")
        return 1

    # 处理前记录原始文件哈希，处理后再比对，证明原图未被覆盖
    before = {p.name: sha256_of(p) for p in files}

    log = []
    ok = True
    for p in files:
        ok &= process_one(p, log)

    after = {p.name: sha256_of(p) for p in files}
    untouched = before == after

    summary = ["", "=" * 78, "[汇总] 原图未被修改：%s" % ("是" if untouched else "否（异常！）"),
               f"[汇总] 任务一整体结果：{'通过' if (ok and untouched) else '存在失败项'}",
               f"[汇总] 输出目录：{OUT_DIR.relative_to(ROOT)}"]
    for line in summary:
        print(line)
    log.extend(summary)

    (OUT_DIR / "run_log.txt").write_text("\n".join(log), encoding="utf-8")
    print(f"[日志] 已写入 {OUT_DIR.relative_to(ROOT) / 'run_log.txt'}")
    return 0 if (ok and untouched) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception:
        # 双击运行时窗口会立刻关闭，所以把错误同时写进 error.txt 备查
        import traceback
        tb = traceback.format_exc()
        print(tb)
        try:
            (ROOT / "error.txt").write_text(tb, encoding="utf-8")
            print(f"[错误已保存] {ROOT / 'error.txt'}")
        except Exception:
            pass
        sys.exit(1)
