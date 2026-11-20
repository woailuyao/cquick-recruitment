"""
Marker 场地标志检测训练脚本

本场地没有点球点，按 3 类训练（不标 P）：
  0 LCross   L 型角点：场地四角 / 禁区角，两条白线的交会顶点
  1 TCross   T 型丁字口：禁区线与底线 / 边线的交点
  2 XCross   X 型交点：中圈与中线相交的两个点

标注框以交点为中心，包含每条分支一小段白线，不要框整条线。
"""

from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 开始训练
model.train(
    data='marker_dataset/data.yaml', # 指向你的配置文件
    epochs=100,
    imgsz=640,
    device='0',   # 用GPU填'0'，用CPU填'cpu'
    batch=16,     # 显存不足可改为8或4
    patience=30   # 如果30轮指标没提升就提前停止，防止过拟合
)
