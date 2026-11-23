# -*- coding: utf-8 -*-
"""
环境自检（对应考核细则 2.3 的检查命令，并额外报告机器规格）

用法：
  D:\\vision_ws\\k1_soccer_vision\\.venv\\Scripts\\python.exe scripts\\env_report.py
"""

import platform
import shutil
import sys


def line(k, v):
    print(f"  {k:<22} {v}")


def main():
    print("=" * 60)
    print("Python 运行环境")
    print("=" * 60)
    line("Python", sys.version.split()[0])
    line("  executable", sys.executable)
    line("  arch", platform.machine())
    line("OS", f"{platform.system()} {platform.release()} ({platform.version()})")

    print()
    print("=" * 60)
    print("关键库")
    print("=" * 60)

    try:
        import cv2
        line("OpenCV", cv2.__version__)
    except Exception as e:
        line("OpenCV", f"FAILED: {e}")

    try:
        import numpy
        line("NumPy", numpy.__version__)
    except Exception as e:
        line("NumPy", f"FAILED: {e}")

    try:
        import torch
        line("PyTorch", torch.__version__)
        line("  CUDA available", torch.cuda.is_available())
        line("  CUDA version", torch.version.cuda)
        line("  threads", torch.get_num_threads())
        line("  device count", torch.cuda.device_count())
    except Exception as e:
        line("PyTorch", f"FAILED: {e}")

    try:
        import ultralytics
        line("Ultralytics", ultralytics.__version__)
    except Exception as e:
        line("Ultralytics", f"FAILED: {e}")

    try:
        import yaml
        line("PyYAML", yaml.__version__)
    except Exception as e:
        line("PyYAML", f"FAILED: {e}")

    try:
        import onnx
        line("onnx", onnx.__version__)
    except Exception as e:
        line("onnx", f"FAILED: {e}")

    try:
        import onnxruntime
        line("onnxruntime", onnxruntime.__version__)
        line("  providers", ", ".join(onnxruntime.get_available_providers()))
    except Exception as e:
        line("onnxruntime", f"FAILED: {e}")

    print()
    print("=" * 60)
    print("外部工具")
    print("=" * 60)
    for exe in ("git", "wsl"):
        p = shutil.which(exe)
        line(exe, p or "NOT FOUND")

    print()
    print("=" * 60)
    print("硬件")
    print("=" * 60)
    try:
        import psutil  # 可选
        line("CPU cores", psutil.cpu_count(logical=True))
        vm = psutil.virtual_memory()
        line("RAM total GB", round(vm.total / 1024 ** 3, 1))
        line("RAM available GB", round(vm.available / 1024 ** 3, 1))
    except Exception:
        import os
        line("CPU cores", os.cpu_count())
        line("RAM", "install psutil for detail")

    return 0


if __name__ == "__main__":
    sys.exit(main())
