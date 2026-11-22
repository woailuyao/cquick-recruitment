"""
冒烟测试：只跑 2 轮，验证数据集读取 / GPU / 前向反向是否正常，不追求指标。
确认无报错后再运行 train_ball.py 正式训练。
"""
from ultralytics import YOLO

DATA_YAML = r"D:\W34_dataset\ball\ball_data.yaml"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\ball"

model = YOLO("yolov8n.pt")
model.train(
    data=DATA_YAML,
    epochs=2,
    imgsz=640,
    batch=8,
    device=0,
    workers=0,          # 冒烟先用 0，避免 Windows 多进程问题
    project=PROJECT,
    name="smoke",
    exist_ok=True,
)
print("smoke train done")
