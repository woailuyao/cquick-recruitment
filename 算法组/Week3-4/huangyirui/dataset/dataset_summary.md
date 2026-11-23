# 数据集统计

2026 年 11 月 18、21、22 日实验室摄像头采集足球素材。首批 21 段共 5,629 帧，扩充训练集纳入新增三段视频。以视频片段划分集合，同段抽帧留在同一集合。足球测试集逐图复核。下表按最终标签文件计数。

| 任务 | 集合 | 标签文件数 | 标注实例 |
|---|---|---:|---|
| ball | train | 358 | ball 360 |
| ball | val | 23 | ball 23 |
| ball | test | 70 | ball 135 |
| marker | train | 38 | LCross 42, TCross 32, XCross 0, PenaltyPoint 0 |
| marker | val | 19 | LCross 16, TCross 22, XCross 0, PenaltyPoint 0 |
| marker | test | 12 | LCross 25, TCross 18, XCross 0, PenaltyPoint 0 |

11 月 23 日的两段视频另用于 7 张伪标签微调实验。最终保留此前标志权重。配置中的 path 需指向本地解包的数据集，图片与标签分别置于 images 和 labels 的对应集合下。
