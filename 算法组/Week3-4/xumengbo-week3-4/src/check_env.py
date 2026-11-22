import sys, traceback
print("python", sys.version.split()[0])

try:
    import ultralytics
    print("ultralytics", ultralytics.__version__)
except Exception:
    print("!! ultralytics import failed")
    traceback.print_exc()

try:
    from ultralytics import YOLO
    print("YOLO import ok")
except Exception:
    print("!! YOLO import failed")
    traceback.print_exc()

for mod in ["yaml", "tqdm", "scipy", "pandas", "requests", "psutil", "polars"]:
    try:
        __import__(mod)
        print(f"dep {mod}: ok")
    except Exception as e:
        print(f"dep {mod}: MISSING ({type(e).__name__})")
