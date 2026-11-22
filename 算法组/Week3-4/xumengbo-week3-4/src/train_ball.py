"""
任务 1 足球检测：YOLOv8n 训练脚本
运行（PyCharm 选 Python 3.10 解释器，或命令行用 3.10 全路径）：
    python train_ball.py
产物输出到：D:\\W3-4_许勐搏_单目定位\\runs\\ball\\train
"""
from ultralytics import YOLO

# ===== 可调参数（v2 重训：降学习率、加大 batch、放宽早停，缓解震荡与欠拟合）=====
EPOCHS = 120          # 训练轮数；放宽到 120，给小数据集足够收敛空间
IMGSZ = 640           # 训练输入分辨率（原图 1280x720 缩放到 640）
BATCH = 16            # 每批图片数；显存实测仅占 1.1G/8G，加到 16 让梯度更稳
WORKERS = 0           # 数据加载子进程数；Windows 用 0 最稳，避免 spawn 多进程报错
PATIENCE = 50         # 早停：连续 50 轮无提升才停，避免把欠拟合的早期权重选为 best
LR0 = 0.005           # 初始学习率；默认 0.01 对本小数据集偏大、易震荡，降到 0.005
DATA_YAML = r"D:\W34_dataset\ball\ball_data.yaml"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\ball"
RUN_NAME = "train_v2"


def main():
    # 1. 加载 nano 基线权重。首次运行会自动下载 yolov8n.pt（约 6.5MB）到缓存目录
    #    yolov8n 是 YOLOv8 系列里最小最快的版本，符合任务要求的 n 型号
    model = YOLO("yolov8n.pt")

    # 2. 开始训练
    #    data      : 数据集配置 yaml（含 path/train/val/test/nc/names）
    #    device=0  : 用第 0 块 NVIDIA 显卡；CPU 训练极慢，不要改
    #    exist_ok  : 允许覆盖同名输出目录
    model.train(
        data=DATA_YAML,
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        device=0,
        workers=WORKERS,
        patience=PATIENCE,
        lr0=LR0,
        project=PROJECT,
        name=RUN_NAME,
        exist_ok=True,
    )

    # 3. 训练结束后，在独立 test 集上评估，出 P / R / mAP
    #    split="test" 指定用 ball_data.yaml 里的 test 划分，不参与训练
    metrics = model.val(
        data=DATA_YAML,
        split="test",
        imgsz=IMGSZ,
        batch=BATCH,
        device=0,
        workers=WORKERS,
        project=PROJECT,
        name="test_eval_v2",
        exist_ok=True,
    )

    # 4. 打印任务书要求的关键指标（单类别，取第 0 类）
    #    metrics.box.p     : Precision 精确率
    #    metrics.box.r     : Recall 召回率
    #    metrics.box.map50 : mAP@0.5
    #    metrics.box.map   : mAP@0.5:0.95
    print("===== test 集指标 =====")
    print(f"Precision (P)      : {metrics.box.p[0]:.4f}")
    print(f"Recall (R)         : {metrics.box.r[0]:.4f}")
    print(f"mAP@0.5            : {metrics.box.map50:.4f}")
    print(f"mAP@0.5:0.95       : {metrics.box.map:.4f}")


if __name__ == "__main__":
    main()
