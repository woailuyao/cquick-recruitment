from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 开始训练，参数与命令行一致
model.train(
    data='data.yaml',
    epochs=10,
    imgsz=640,
    device='cpu'
)
