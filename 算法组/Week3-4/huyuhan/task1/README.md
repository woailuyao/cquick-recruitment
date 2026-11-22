# 任务一：足球检测（YOLOv8）交付说明

## 1. 项目概述

本任务使用 YOLOv8 目标检测模型识别画面中的足球，类别为单类
`ball`。本交付目录保存正式模型权重、训练日志、训练/验证可视化、模型选择记录、独立评估结果及典型错误案例。

**正式模型：** YOLOv8s\
**权重文件：** `weights/best.pt`\
**权重 SHA256：**

``` text
732ecee9f41639b6888632e1cd8417569807f570a680423b29547c9752824eae
```

## 2. 目录结构

``` text
task1/
├── README.md
├── weights/
│   └── best.pt
├── training/
│   ├── args.yaml
│   ├── results.csv
│   ├── training_analysis.md
│   └── training_summary.json
├── visualizations/
│   ├── results.png
│   ├── BoxPR_curve.png
│   ├── BoxP_curve.png
│   ├── BoxR_curve.png
│   ├── BoxF1_curve.png
│   ├── confusion_matrix.png
│   ├── confusion_matrix_normalized.png
│   ├── labels.jpg
│   ├── train_batch*.jpg
│   └── val_batch*_labels.jpg / val_batch*_pred.jpg
├── evaluation/
│   ├── evaluation_audit.md
│   ├── selection_before_test.json
│   ├── validation_precision_matrix.json
│   ├── val_n/
│   │   ├── summary.json
│   │   └── details.json
│   ├── val_s/
│   │   ├── summary.json
│   │   └── details.json
│   └── final_test/
│       ├── summary.json
│       └── details.json
└── error_cases/
    └── *_review.jpg
```

**交付范围说明：**

本交付根目录包含独立的 `dataset/` 与 `reports/` 目录。本 README 位于 `task1/`，关联材料使用上级目录相对路径。
原文列出的数据来源修复记录及错误分析报告，在当前暂存目录
和原项目检索范围内均未找到，因此不将其列为已交付文件。

若验收要求这些材料，应从原始数据与审计记录重新生成、
复核后再补入。当前可核验材料以本目录实际存在的
`evaluation/`、`error_cases/` 和 `training/` 为准。

## 3. 数据集与划分

  子集          图片数
  ---------- ---------
  Train            514
  Val              145
  Test              76
  **合计**     **735**

-   检测类别数：1。
-   类别映射：`0: ball`。
-   图像尺寸：1280 × 720。
-   Test 中有 76 张图片，74 个真实目标实例；其中 2 张为空标签背景图。
-   Test 图片均来自视频 `WIN_20261108_15_54_49_Pro.mp4`。

数据一致性审计已核对 Test 与冻结计划中的图片、标签、SHA256
和图像尺寸；Val 也已完成同类核对。审计结果为图片/标签缺失 0、哈希不一致
0、尺寸不一致 0、标签内容不一致 0。冻结计划 SHA256：

``` text
5de8ce62cd171d54dff06cf7dd81ee6889facc35f140292ce7eae01e5880e2a8
```

**数据范围说明：**
测试图片来自单一视频。实验记录同时指出，虽然视频文件按划分隔离，但实验室/拍摄场景相同，相关素材此前也曾在开发过程中被观察。因此，这组
Test 结果不能直接等同于跨场景、跨设备或外部数据集上的泛化性能。

## 4. 环境与训练配置

已记录的训练环境： - Ubuntu 22.04（WSL） - Python 3.10.12 - PyTorch
2.9.1+cu126 - CUDA 12.6 - NVIDIA GeForce RTX 4050 Laptop GPU（约 6 GB
显存） - Ultralytics 8.4.208

正式训练从 COCO 预训练权重初始化。模型比较包含 YOLOv8n 与
YOLOv8s；主要训练设置如下：

  参数                    设置
  ----------------------- ----------------
  Epochs                  100
  Patience                20
  Image size              640
  Batch size              4
  Optimizer               AdamW
  Initial learning rate   0.001
  Final LR factor         0.01
  Workers                 2
  Seed                    42
  Deterministic           True
  AMP                     True
  Mosaic                  0.5
  Close mosaic            最后 10 epochs
  MixUp / Copy-Paste      0 / 0

完整训练参数以 `training/args.yaml` 为准。

## 5. 模型选择与冻结

模型选择规则：**只根据 Val 的 mAP50-95
选择模型；若相同则选择较小模型。模型选择记录在测试开始前保存，之后才运行
Test。**

冻结选择记录：

  模型        Val mAP50-95
  --------- --------------
  YOLOv8n     0.9179028656
  YOLOv8s     0.9309796305

因此选择 YOLOv8s。`selection_before_test.json` 中记录的权重 SHA256
与最终 `weights/best.pt` 的实际哈希一致。Test
评估指纹也对应同一权重哈希和同一冻结数据计划。

## 6. 最终 Test 结果

### 6.1 Ultralytics 原生 AP 评估

  指标              Test
  ----------- ----------
  Precision     1.000000
  Recall        0.998792
  mAP50         0.995000
  mAP50-95      0.947860

原生评估使用低置信度阈值进行 AP 计算。这里的原生 Precision/Recall 来自
Ultralytics 选定的工作点，**不等于固定置信度阈值 0.25 或 0.50 下的
Precision/Recall**。

### 6.2 固定置信度阈值评估

固定阈值统计采用类别 0，预测框与真实框按 IoU ≥ 0.50 进行匹配。

    Confidence threshold   TP   FP   FN   Precision     Recall         F1
  ---------------------- ---- ---- ---- ----------- ---------- ----------
                    0.25   74    5    0    0.936709   1.000000   0.967320
                    0.50   74    2    0    0.973684   1.000000   0.986667
                    0.90   73    0    1    1.000000   0.986486   0.993197
                    0.95   25    0   49    1.000000   0.337838   0.505051
                    0.97    0    0   74         N/A   0.000000   0.000000
                    0.99    0    0   74         N/A   0.000000   0.000000

**阈值使用建议：** - 若优先保证不漏检，可从 0.50
作为候选工作阈值开始验证：本次 Test 上 Recall 为 1.0，Precision 为
0.973684。 - 若更重视减少误检，可评估 0.90：本次 Test 上没有 FP，但漏掉
1 个真实目标。 - 0.95 及以上会明显损失召回率；0.97/0.99
下没有检出框，不适合作为当前模型的部署阈值。

阈值应结合实际摄像头画面、运行环境和任务对漏检/误检的代价进一步确认。离线图片测试不能替代真实摄像头端到端测试。

## 7. 典型错误案例

交付目录 `error_cases/` 保存了 6 张复核图： - 1
张低置信度漏检案例：真实足球置信度约 0.8793，在阈值 0.90 下被过滤。 - 5
张阈值 0.25 下的误检案例：误检置信度分别约为
0.3638、0.7291、0.6610、0.4287、0.4245。 - 阈值提高到 0.50
后，仍保留的误检为置信度约 0.7291 和 0.6610 的两例。 -
复核图显示，画面边缘的高亮白色物体/结构可能产生额外检测框；球靠近白色绳网等复杂边界时，置信度也可能下降。

## 8. 训练指标口径说明

`training/results.csv` 的最佳轮次为 Epoch 87，记录的 Val mAP50-95 为
0.93309。冻结模型选择记录中的 YOLOv8s Val mAP50-95 为 0.9309796305。

这两个数值来自不同的记录/评估流程，不应混写为同一个结果。后续按冻结评估设置进行的
batch=1、FP32 验证复现了 0.9309796305；另行进行的 FP16 或不同 batch
设置得到约 0.93315。不同数值精度/批量设置的结果已记录在
`evaluation/validation_precision_matrix.json`。模型选择仍以测试前冻结记录为准，不因测试后复测结果改变模型选择。

## 9. 交付与复现边界

本目录包含正式权重、训练参数与日志、可视化、评估汇总和逐图详情、错误案例及数据选择审计材料。

**当前暂存包并非完全自包含的从零复现实验包。**
原始图片/标签数据、完整训练/评估源码、`plan_v3.json`、数据集 `data.yaml`
和初始 COCO 权重仍需从原始 Cquick
项目目录配套归档。若验收要求在另一台机器上从头复现，应将这些源文件、依赖版本和运行命令一并提供。

此外，评估脚本明确标注目录图片吞吐率不是摄像头
FPS；目前不能仅凭离线评估结果宣称已达到实时摄像头帧率要求。

## 10. 关键文件

-   正式权重：`weights/best.pt`
-   训练参数：`training/args.yaml`
-   训练日志：`training/results.csv`
-   训练曲线：`visualizations/results.png`
-   模型选择冻结记录：`evaluation/selection_before_test.json`
-   Val-N / Val-S 评估：`evaluation/val_n/`、`evaluation/val_s/`
-   最终 Test 评估：`evaluation/final_test/`
-   数据评估审计：`evaluation/evaluation_audit.md`
-   典型错误案例：`error_cases/`


## 演示视频

- CQuick 算法组第 3–4 周考核：任务一足球检测

  https://www.bilibili.com/video/BV1yesU6rEMp/


### 根目录关联材料

- [数据来源修复审计](../dataset/selection_v3_source_repair_audit.md)
- [修复后的数据选择清单](../dataset/selection_v3_source_repaired.csv)
- [任务一技术报告](../reports/task1_technical_report.md)
- [任务一错误案例分析](../reports/task1_error_analysis.md)
- [任务一可复现源码说明](../task1_reproducibility_source_20261122/rebuilt_yolo_ball_v3/README.md)

## 2026-11-23 正式权重校验与提交说明

正式模型：YOLOv8s。

正式训练权重本机源文件：
`/home/hu_yuhan/Cquick/Week3-4/results/task1_ball_yolo/formal_v3_20261114_214759_977028/ball_yolov8s_20261115_195246_886751/weights/best.pt`

文件大小：22,500,650 bytes。

SHA256：
`732ecee9f41639b6888632e1cd8417569807f570a680423b29547c9752824eae`

本权重不纳入 Git 二进制提交。该路径是原始训练环境中的本机路径，不是公共下载地址。权重共享存储链接尚未配置，不能将本机路径描述为公开下载链接。

B站实时演示：
https://www.bilibili.com/video/BV1yesU6rEMp/
