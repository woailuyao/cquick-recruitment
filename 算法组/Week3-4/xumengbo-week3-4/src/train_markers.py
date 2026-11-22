"""
任务 2 场地标志检测：YOLOv8n 训练脚本（LCross / TCross / XCross）
=====================================================================
前置条件：D:\\W34_dataset\\markers 下已完成人工标注（labels 目录有对应 txt）。
         未标注前不要运行：没有标签，训练无法出有效指标。

运行（PyCharm 选 Python 3.10，或命令行用 3.10 全路径）：
    python train_markers.py

产物输出到：D:\\W3-4_许勐搏_单目定位\\runs\\markers\\train
评估输出到：runs\\markers\\test_eval
"""
from ultralytics import YOLO

# ===== 可调参数（与足球模型同一套小数据集经验值）=====
EPOCHS = 120          # 训练轮数；小数据集给足收敛空间
IMGSZ = 640           # 训练输入分辨率（原图 1280x720 缩放到 640）
BATCH = 16            # 每批图片数；显存不足就降到 8 或 4
WORKERS = 0           # Windows 用 0 最稳，避免 spawn 多进程报错
# 注意：当前 val（clip_05）有 49 个 TCross 框，但全是门柱旁 T 型特写，无 L/X 样本，
#       验证指标只反映 T 类，不能全面代表三类泛化；现有 best.pt 是早期空 val 下训练的。
#       待回场地补拍开阔场地多类标志、扩充 val 后再正式重训；
#       重训时可把 patience 改回 30~50，按 val mAP 正常早停并选 best。
PATIENCE = 0          # 早停：0 表示关闭早停；当前 val 仅 T 类，暂不按验证指标提前停
LR0 = 0.005           # 初始学习率；小数据集偏大易震荡，先取 0.005
DATA_YAML = r"D:\W34_dataset\markers\markers_data.yaml"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\markers"
RUN_NAME = "train"


def main():
    # 1. 加载 nano 基线权重；首次运行自动下载 yolov8n.pt
    model = YOLO("yolov8n.pt")

    # 2. 训练。data 指向标志数据集 yaml（path/train/val/test/nc/names）
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

    # 3. 在独立 test 集评估，出逐类 P / R / mAP50（细则 2.4 要求分类别报告）
    metrics = model.val(
        data=DATA_YAML,
        split="test",
        imgsz=IMGSZ,
        batch=BATCH,
        device=0,
        workers=WORKERS,
        project=PROJECT,
        name="test_eval",
        exist_ok=True,
    )

    # 4. 逐类打印 P / R。metrics.box.p/r 是按类别编号排列的数组
    #    model.names: {0: 'LCross', 1: 'TCross', 2: 'XCross'}
    #    逐类 mAP50 不同版本属性名不一致，这里整体 mAP50 见下方；
    #    逐类 mAP 可在 runs/markers/test_eval 的 results 曲线里核对。
    print("===== test 集逐类指标 =====")
    names = model.names
    for cls_id, cls_name in names.items():
        p = float(metrics.box.p[cls_id])
        r = float(metrics.box.r[cls_id])
        print(f"{cls_name:8s}  P={p:.4f}  R={r:.4f}")

    # 整体指标（所有类别合并）
    print(f"\n整体 mAP@0.5   : {metrics.box.map50:.4f}")
    print(f"整体 mAP@0.5:0.95: {metrics.box.map:.4f}")
    print(f"\n最佳权重: {PROJECT}\\{RUN_NAME}\\weights\\best.pt")


if __name__ == "__main__":
    main()
