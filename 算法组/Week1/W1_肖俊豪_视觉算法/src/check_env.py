# -*- coding: utf-8 -*-
"""
运行环境自检：Python 版本、numpy、opencv、测试图片、输出目录。

用法：python src/check_env.py
"""

import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs"

REQUIRED_IMAGES = [
    "01_blue_cube_gray.jpg",
    "02_green_bottle_wood.jpg",
    "03_orange_ball_white.jpg",
    "04_hard_orange_shadow.jpg",
]


def line(msg=""):
    print(msg)


def main():
    ok = True
    line("=" * 66)
    line("视觉算法第一周 · 环境自检")
    line("=" * 66)

    # ---------- 1. Python 版本 ----------
    v = sys.version_info
    line(f"[Python ] {v.major}.{v.minor}.{v.micro}  ({sys.executable})")
    if v < (3, 8):
        line("[错误  ] 需要 Python 3.8 及以上，建议 3.10")
        ok = False

    # ---------- 2. 依赖 ----------
    need = []
    try:
        import numpy as np
        line(f"[numpy  ] {np.__version__}  OK")
    except Exception as e:
        line(f"[错误  ] numpy 导入失败：{type(e).__name__}: {e}")
        need.append("numpy")

    try:
        import cv2
        line(f"[opencv ] {cv2.__version__}  OK")
    except Exception as e:
        line(f"[错误  ] opencv 导入失败：{type(e).__name__}: {e}")
        need.append("opencv-python")

    if need:
        ok = False
        line("")
        line("修复方法（复制这一行执行，它用的是当前这个 Python，不会装错地方）：")
        line(f'    "{sys.executable}" -m pip install {" ".join(need)}')
        line("如果下载慢，可加国内镜像：")
        line(f'    "{sys.executable}" -m pip install {" ".join(need)} '
             f'-i https://pypi.tuna.tsinghua.edu.cn/simple')
        line("用 Anaconda 的话也可以：conda install -c conda-forge opencv numpy")
        line("")

    # ---------- 3. 测试图片 ----------
    if not IMG_DIR.exists():
        line(f"[错误  ] 找不到目录：{IMG_DIR}")
        ok = False
    else:
        missing = [n for n in REQUIRED_IMAGES if not (IMG_DIR / n).exists()]
        n_have = len(list(IMG_DIR.glob("*.jpg"))) + len(list(IMG_DIR.glob("*.png")))
        if missing:
            line(f"[图片  ] 缺少 {len(missing)} 张：{', '.join(missing)}")
            line("          可运行 python src/make_test_images.py 重新生成")
            ok = False
        else:
            line(f"[图片  ] data/images 下共 {n_have} 张，测试图齐全  OK")

    # ---------- 4. 输出目录 ----------
    try:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        probe = OUT_DIR / ".write_test"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink()
        line(f"[输出  ] outputs/ 可写  OK")
    except Exception:
        line("[错误  ] outputs/ 不可写（权限或只读盘？）")
        traceback.print_exc()
        ok = False

    # ---------- 结论 ----------
    line("-" * 66)
    if ok:
        line("环境正常，可以依次运行：")
        line("    python src/task1_basics.py")
        line("    python src/task2_processing.py")
        line("    python src/task3_color_detect.py --sweep")
        line("或直接双击项目根目录的 run_all.bat")
    else:
        line("环境不满足要求，请先按上面的提示处理。")
    line("=" * 66)
    return 0 if ok else 1


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
