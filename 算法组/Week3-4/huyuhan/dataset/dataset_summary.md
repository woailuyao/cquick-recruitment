# CQuick 第3–4周数据集汇总（提交用）

> 数据基于已提交的实验 README 汇总，**本补交程序没有重新统计全体原始图片或重新训练模型**。应以本机冻结清单、原始标签和数据划分核对。

## 任务一：足球

- 类型：本人实验室采集，YOLO 单类 `ball`；
- 划分：Train 514 份，Val 145 份，Test 76 份，合计 735 份（包含训练派生/重复采样，不能表述为 735 张独立拍摄图）；
- Train/Val/Test 按视频源隔离，独立测试来源为一个视频；
- 训练参数、独立集指标和误检/漏检详见 `../task1/README.md`。

## 任务二：L/T/X 场地标志

- 冻结正式数据：Train 73 张（10 个视频源），Val 23 张（3 个视频源），共 96 张、13 段独立视频；
- 类别实例：LCross 134，TCross 130，XCross 43，合计 307 个标注实例；
- PenaltyPoint：现场没有可见 GT，因此当前有效标注数为 0；
- 96 张低于细则建议的 300 张，**307 个标注实例不等于 307 张图片**；
- 未有可确认的独立最终测试集分数；现有指标是固定验证集结果。

## 数据及权重交付边界

组织方明确要求**不要将整套抽帧图片/标注图片或 .pt 权重直接纳入 Git**，按规则保存于本机或指定共享存储，并在 README 提供真实可访问的权重获取方式和哈希。
`ball_data.yaml` / `marker_data.yaml` 为源工作区的数据配置副本，仅在相关图片数据存在、路径修正后可用于重训。

## 本次自动检查记录

```json
{
  "ball_data.yaml": "Copied from existing Task1 data.yaml; paths may point to unavailable external data.",
  "marker_data.yaml": "Copied from actual original Task2: /home/hu_yuhan/Cquick/Week3-4/site_review_20261119/formal_dataset/data.yaml. Its local paths must be adapted after dataset download."
}
```
