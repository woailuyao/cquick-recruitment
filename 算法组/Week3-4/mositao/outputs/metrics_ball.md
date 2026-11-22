# 目标检测 评估指标（ball）

- 权重：`Week3-4/weights/best_ball.pt`
- 数据：`Week3-4/dataset/data.yaml`（split = test）
- 置信度阈值：0.25

| 指标 | 数值 | 含义 |
| --- | ---: | --- |
| Precision | 1.0000 | 检出框中真正是目标的比例 |
| Recall | 0.9388 | 真实目标被检出的比例 |
| mAP50 | 0.9350 | IoU=0.5 时的平均精度 |
| mAP50-95 | 0.8804 | 多 IoU 阈值下的平均精度 |

> 测试集不参与训练与验证；具体划分方式见 `dataset_summary.md`。

## 案例

- 误检案例 `Week3-4/outputs/cases_ball/fp_*.png`：红框为误检（模型认为有目标但人工没标）
- 漏检案例 `Week3-4/outputs/cases_ball/fn_*.png`：橙框为漏检（人工标了但模型没检出）
- 对照案例 `Week3-4/outputs/cases_ball/tp_*.png`：检出正确

分析要点（写 README 时展开）：
- 误检多来自白色场地线、圆形杂物（球鞋、标志盘）——它们的局部纹理与球相似
- 漏检多发生在球贴画面边缘（只露一半）、强反光、以及小目标（面积占比 <0.2%）
