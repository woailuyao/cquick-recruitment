"""
标志模型 · 1 轮 smoke test（训练前自检）
=========================================
目的：只跑 1 个 epoch，确认
  1. train/val/test 数据集能被 ultralytics 正常加载；
  2. 空 val（clip_05 素材缺陷、标签已隔离）阶段是否报错；
  3. 产物目录与权重能正常写出。
跑完看终端日志与 D:\\W3-4_许勐搏_单目定位\\runs\\markers\\smoke。

用法：python smoke_markers.py
"""
from ultralytics import YOLO

DATA_YAML = r"D:\W34_dataset\markers\markers_data.yaml"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\markers"

def main():
    # 加载 nano 基线权重；本地没有时会自动联网下载 yolov8n.pt
    model = YOLO("yolov8n.pt")

    # 仅 1 轮、小 batch，只为验证流程，指标不具参考意义
    model.train(
        data=DATA_YAML,
        epochs=1,
        imgsz=640,
        batch=4,
        device=0,
        workers=0,
        project=PROJECT,
        name="smoke",
        exist_ok=True,
    )
    print("smoke done. 产物目录:", PROJECT, "\\smoke")

if __name__ == "__main__":
    main()
