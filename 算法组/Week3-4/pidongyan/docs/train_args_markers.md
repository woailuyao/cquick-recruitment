# 训练记录 markers

- 时间：2026-11-21 22:58:01
- preset：`baseline`
- 数据：`config/marker.yaml`
- 基线：yolov8n.pt / imgsz=640 / epochs=50 / batch=auto / device=0

## 本次全部参数

| 参数 | 值 | 与基线不同 |
|---|---|---|
| batch | auto | 否 |
| device | 0 | 否 |
| epochs | 50 | 否 |
| imgsz | 640 | 否 |
| model | yolov8n.pt | 否 |
| patience | 30 | 否 |
| seed | 0 | 否 |
| workers | 4 | 否 |

## 独立测试集结果

| 指标 | 值 |
|---|---|
| Precision | 0.9395 |
| Recall | 0.0500 |
| mAP50 | 0.1129 |
| mAP50-95 | 0.0338 |

> best.pt 由 **val** 集选出，test 集仅用于最终报告，未用于挑模型或调参。
