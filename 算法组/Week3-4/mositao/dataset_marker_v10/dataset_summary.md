# 场地标志数据集统计

## 划分（按视频段整体划分，避免相邻帧泄漏）

| 划分 | 视频段 | 图片 | 框 | 类别分布 |
| --- | --- | ---: | ---: | --- |
| train | marker_low_01,marker_low_03,marker_s01,marker_s03 | 211 | 1264 | LCross=529 TCross=515 XCross=220 |
| val | marker_s04 | 63 | 99 | LCross=0 TCross=99 XCross=0 |
| test | marker_low_02,marker_s02,marker_s05 | 193 | 390 | LCross=201 TCross=114 XCross=75 |

合计 1753 个框。

## 说明

- 类别顺序：`LCross=0, TCross=1, XCross=2, PenaltyPoint=3`。
- 未标注（无 .txt）的图片已排除；空 .txt 视为明确的负样本保留。
- 各段拍摄对象不同（s02 近景中圈交点、s04 遮挡、s05 另一光照），
  因此单段内类别较单一，划分时已尽量让 train/val/test 各含多类。
