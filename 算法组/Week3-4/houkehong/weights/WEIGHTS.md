# 权重文件获取方式与校验值

> 按细则「五、提交规范」，`best.pt` 等较大权重不提交到 GitHub，
> 放在实验室共享存储；本文件提供校验值供核对。

| 文件 | 大小 | SHA256 | 用途 |
|---|---:|---|---|
| `ball_best.pt` | 6.18 MB | `0580f20af9e9436fce805b8db8444ccb84196aa3c464d833bc6018fca8c68fe5` | 任务 1 足球检测 |
| `marker_best.pt` | 6.18 MB | `05cf6140e8087d6876883648a20de64b51b9b40ea7b59e4f699b8b4270ab15f2` | 任务 2 场地标志检测（v6） |
| `ball_best.onnx` | 12.06 MB | `311beb03746d53ba798dcdf46de333db2c7e4ca2e4bbd2e799c2bcdcb3d2783e` | 任务 1 ONNX 导出 |
| `marker_best.onnx` | 12.16 MB | `af85cc6c3d10c44f939ee1cb7b87122ba3c7edb69fafc611e86f0e9b498f98ae` | 任务 2 ONNX 导出 |

## 获取方式

权重由本人训练产出，已放入**实验室共享存储**（路径向评审单独说明）。
如需自行复现，可按下述命令训练：

```bash
# 任务 1
yolo detect train model=yolov8n.pt data=datasets/ball/data.yaml \
     epochs=30 imgsz=640 batch=32 seed=0

# 任务 2（从上一版微调，固定划分见 dataset/dataset_summary.md）
yolo detect train model=marker_start.pt data=datasets/marker_v6/data.yaml \
     epochs=40 imgsz=640 batch=32 seed=0
```
