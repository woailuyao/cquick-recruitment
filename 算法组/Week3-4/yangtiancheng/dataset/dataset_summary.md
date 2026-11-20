# 数据集说明

整理人：yangtiancheng  
完成日期：2026-11-20

所有视频均为本人实验室实拍。提交包不包含原始视频、图片、标签文件或模型权重，只保留数据配置与统计说明。

## 1. 任务一：足球检测数据

数据集名称：`ball_split_v3`  
数据集根目录（虚拟机）：`/home/obsession/vision_ws/k1_soccer_vision/datasets/ball_split_v3`  
类别：`0=ball`

| split | 图片/标签 | 有球图片 | 无球背景 |
|---|---:|---:|---:|
| train | 275 | 160 | 115 |
| val | 67 | 33 | 34 |
| test | 77 | 36 | 41 |
| 合计 | 419 | 229 | 190 |

视频段数与划分方式：

| split | 有球来源视频 | 无球来源视频 |
|---|---|---|
| train | 2, 3, 4, 5, 6, 9, 10, 11 | 15 |
| val | 12 | 16 |
| test | 1, 14 | 17 |

- 实际使用 14 段独立视频来源。
- 整段视频只进入一个 split，不混分同一视频相邻帧。
- 有球来源只复制已有非空标签的图片与标签配对，未标注抽帧不参与 `ball_split_v3`。
- 无球来源要求每张图片都有空标签。
- `test` 的有球来源 1、14 未参与训练。

说明：`ball_split_v3` 原始数据只保存在 cquick Ubuntu 虚拟机内；提交包组装时虚拟机未运行，Windows 工作区没有该数据目录。上表数字来自 2026-11-12 数据集构建脚本的成功输出及会话记录。无法在当前主机重新逐文件统计，因此没有把其他 split 的数字冒充为 test。

随包配置：[data.yaml](ball/data.yaml)

该 YAML 按 `build_ball_split_v3.py` 的原始生成格式复原；原始 `ball.yaml` 位于虚拟机路径：

```text
/home/obsession/vision_ws/k1_soccer_vision/datasets/ball_split_v3/ball.yaml
```

## 2. 任务二：场地标志数据

基础训练数据集：`marker_dataset`  
当前 Windows 路径：`C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\task2\marker_dataset`

下列数字由当前实际图片和标签文件重新统计：

| split | 图片 | 标签 | 空标签/背景 | LCross | TCross | XCross | PenaltyPoint |
|---|---:|---:|---:|---:|---:|---:|---:|
| train | 150 | 150 | 99 | 28 | 3 | 20 | 0 |
| val | 82 | 82 | 24 | 0 | 58 | 0 | 0 |
| test | 40 | 40 | 5 | 35 | 0 | 0 | 0 |
| 合计 | 272 | 272 | 128 | 63 | 61 | 20 | 0 |

视频划分：

| 视频 | split |
|---|---|
| `01_normal.avi` | train |
| `02_glare.avi` | test |
| `03_goalpost.avi` | val |
| `04_invalid.avi` | train |

- 当前活动训练数据来自 4 段独立视频。
- `04_invalid.avi` 为纯负样本段，强制进入 train。
- 抽帧总清单为 321 张。
- `22.avi` 的 49 张中心点帧未进入当前 L/T/X 训练轮次；排除后活动数据为 272 张。
- `PenaltyPoint` 在当前活动训练数据中为 0 实例，因此本轮没有训练该类别。

随包配置：[data.yaml](marker/data.yaml)

## 3. 独立 XCross 评测集

数据集：`marker_eval_xtest_20261118`

- 来源：额外拍摄的 `23.avi`
- 抽帧：73 张
- 正样本：31 张，共 31 个 XCross 实例
- 无目标背景：42 张
- 使用方式：整段只作为 test
- 未改动原 `marker_dataset` 的 train/val/test 划分
- 未重新训练

LabelImg 因 `classes.txt` 多出一行而把人工框保存成类别 4；构建评测集时仅在副本中映射为 `2=XCross`，框坐标未修改。

## 4. 数据安全说明

- 提交包只包含 `data.yaml` 和统计说明。
- 不包含任何 `.pt`、视频、原始数据图片、YOLO 标签或大数据文件。
- 数据配置来源和不可访问项已在各节明确标注。
