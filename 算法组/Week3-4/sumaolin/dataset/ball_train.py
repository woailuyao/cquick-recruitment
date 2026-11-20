from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 开始训练
model.train(
    data='ball_dataset/data.yaml', # 指向你的配置文件
    epochs=100,
    imgsz=640,
    device='0',   # 用GPU填'0'，用CPU填'cpu'
    batch=16,     # 显存不足可改为8或4
    patience=30   # 如果30轮指标没提升就提前停止，防止过拟合
)