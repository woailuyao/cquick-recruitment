"""
录屏用：在终端展示任务 1 模型信息 + 独立 test 集指标 + 误检/漏检案例分析。
运行（Python 3.10 全路径，或 PyCharm 选 3.10 SDK）：
    python show_metrics.py

说明：
- 重新在 test 划分上跑一次 model.val()，指标来自当前 best.pt，不是训练时的 val 虚高值；
- 案例分析文字对应 test_eval_v2/val_batch*_pred.jpg 预测图里实际观察到的现象。
"""
from ultralytics import YOLO

WEIGHTS = r"D:\W3-4_许勐搏_单目定位\runs\ball\train_v2\weights\best.pt"
DATA_YAML = r"D:\W34_dataset\ball\ball_data.yaml"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\ball"
RUN_NAME = "test_eval_show"
IMGSZ = 640
BATCH = 16


def main():
    # 1. 加载训练好的最佳权重
    model = YOLO(WEIGHTS)

    # 2. 打印模型基本信息（层数、参数量、类别名）
    print("===== 模型信息 =====")
    print(f"权重文件: {WEIGHTS}")
    # ultralytics 新版：内部 DetectionModel 的层列表在 model.model.model
    inner = model.model
    n_layers = len(inner.model) if hasattr(inner, "model") else len(list(inner.children()))
    print(f"模型结构: {n_layers} 层")
    n_params = sum(p.numel() for p in inner.parameters())
    print(f"参数量  : {n_params:,}")
    print(f"类别    : {model.names}")

    # 3. 在独立 test 集上评估（test 不参与训练/调参）
    metrics = model.val(
        data=DATA_YAML,
        split="test",
        imgsz=IMGSZ,
        batch=BATCH,
        device=0,
        workers=0,
        project=PROJECT,
        name=RUN_NAME,
        exist_ok=True,
    )

    # 4. 打印任务书要求的四项指标（单类别，取第 0 类）
    print("===== 独立 test 集指标 =====")
    print(f"Precision  (P)     : {metrics.box.p[0]:.4f}  "
          f"(判为球的结果里真球占比；越高误检越少)")
    print(f"Recall     (R)     : {metrics.box.r[0]:.4f}  "
          f"(真球里被检出的占比；越高漏检越少)")
    print(f"mAP@0.5            : {metrics.box.map50:.4f}")
    print(f"mAP@0.5:0.95       : {metrics.box.map:.4f}")

    # 5. 误检 / 漏检案例分析（对照 test_eval_v2 预测图）
    print("===== 案例分析 =====")
    print("[误检] 早期 v1 曾把地面白色场地标线、浅色光斑误判为足球，出现空框；")
    print("       v2 降低学习率、加大 batch 后，白线空框基本消失，残余误检多为")
    print("       与球颜色相近的浅色小块，置信度偏低。")
    print("[漏检] 远处小尺寸球、被人腿/门柱/网遮挡的帧，以及运动模糊帧，")
    print("       容易低置信或直接漏检；走动中贴近画面边缘的球也会丢失。")
    print("[原因] 实拍样本量小、val 与 test 场景分布差异较大（domain gap），")
    print("       远景与遮挡样本覆盖不足，是当前 Recall 偏低的主要原因。")


if __name__ == "__main__":
    main()
