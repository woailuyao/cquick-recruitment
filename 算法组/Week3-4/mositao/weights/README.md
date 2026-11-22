# weights/ 目录说明

本目录放**预训练初始权重**与训练产出的**最佳权重**。
按细则 §五，权重不进 Git；本文件只记录文件名、大小与 SHA256，供评审核对。

> 获取方式：两份正式权重（`best_ball.pt`、`best_marker_v11.pt`）随交付一并放入
> 实验室共享存储 / 指定网盘，目录与文件名保持一致；也可用 `tools/train.py`
> 按第 9 节的复现命令自行训练得到。**下载后请核对下面的字节数与 SHA256**。

## 一、预训练初始权重

| 文件 | 大小 | SHA256 |
| --- | ---: | --- |
| `yolov8n.pt` | 6 534 387 字节（6.23 MB） | `31e20dde3def09e2cf938c7be6fe23d9150bbbe503982af13345706515f2ef95` |

- 来源：Ultralytics 官方发布的 YOLOv8n COCO 预训练权重；
- 本机网络下 GitHub 直连被拦，实际走 HuggingFace 镜像：

```bash
curl -L --retry 3 -o weights/yolov8n.pt \
  https://hf-mirror.com/Ultralytics/YOLOv8/resolve/main/yolov8n.pt
```

- 校验（必须与上表一致，否则文件被截断）：

```bash
python -c "from src.common.config_io import sha256_of; print(sha256_of('weights/yolov8n.pt'))"
```

> 实测教训：`ghproxy.net` 首次下载被截断为 5 024 654 字节，`torch.load` 报
> `failed finding central directory`。镜像下载**必须核对字节数或 SHA256**，
> 不能只看「下载成功」。

## 二、正式交付权重

| 任务 | 文件 | 大小 | SHA256 |
| --- | --- | ---: | --- |
| 任务一 足球检测 | `best_ball.pt` | 6 483 434 字节 | `14b92de102e4f8e5796f3a17b73411a02e943d4cdc5d6f6ca1973968c3eff01e` |
| 任务二 场地标志 | `best_marker_v11.pt` | 6 485 610 字节 | `e47fb21d5b043e2f0f4e749f95b6884075d367e09388723af6533c2b821234a5` |

- 训练入口：`tools/train.py`（训完自动把 `runs/<name>/weights/best.pt` 复制成
  `weights/best_<name>.pt`）；`runs/` 整个训练目录不提交；
- 任务二用 v11 而非 v9/v12 的理由（现场召回对比）见根目录 README 4.3。

## 三、对照实验权重（可选，用于复现 4.3 节实验）

| 文件 | 大小 | SHA256 | 说明 |
| --- | ---: | --- | --- |
| `best_marker_v10.pt` | 6 485 610 字节 | `85fd638d556e47e4dfe1ccf7e9ac3ae9995e248b1b39a5a3cb0b7d2297422407` | v11 的前一版 |
| `best_marker_v12.pt` | 6 486 890 字节 | `7dfb974d77e7a509818d814fa360ab60864ae97921fb673e3676bd305de0a6e8` | 干净数据对照，低姿 test 指标最高但现场最差 |
| `best_marker_v9.pt` | 6 485 738 字节 | `3ff990c4ae31d840755a40d47210ce8ff0761665ffe734eab259ef141852bb86` | 旧版（mAP50 0.5826，段内划分偏乐观） |

其余 `best_marker_v2..v8.pt`、`best_marker_v4_keepP.pt`、`best_ball_wrongcopy_marker_v1.pt`
是历史版本或误命名的文件，**不属于交付物**，本地留存即可。
