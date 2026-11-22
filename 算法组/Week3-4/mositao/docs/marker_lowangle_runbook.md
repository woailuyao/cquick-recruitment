# 低机位（0.73 m）标志模型补数据 · 操作手册

> 背景：相机从 1.375 m 降到 0.73 m 后，掠射视角让标志交点的形状偏离训练分布，
> v9 模型在同一位置的置信度只有 0.06–0.22（阈值 0.3 时一个框都不给）。
> 处置：补拍低角度素材、微调模型，再重解外参。

## 现状（2026-11-21 实测）

| 项 | 结果 |
| --- | --- |
| v9 在 0.73 m 截图上的表现 | conf ≥0.3 无框；conf 0.02 时 3 个交点位置正确、置信度 0.10–0.22 |
| `prelabel_marker.py`（几何法） | **不可用**：4 个框全落在凳子腿和球门网上 |
| `prelabel_marker_lowconf.py`（低置信模型法） | **可用**：4 个种子落在真实交点上，只需人工改类别 |

## 一、拍摄（阁下操作，约 3 分钟素材）

机位保持 0.73 m、俯角 11–13°，**不要动对焦环**。

| 段 | 内容 | 要点 |
| --- | --- | --- |
| low_01 | 慢速水平摇：从中圈一侧摇到另一侧 | 每 40 s 只摇 60–90°，越慢越好 |
| low_02 | 对着远端球门端：罚球区/球门区那几个 T、L | 让交点大小有明显变化（近大远小） |
| low_03 | 斜 45° 看场地，包含中圈两个 X 交点 + 一个角点 | 补长条形、不对称形状 |

```powershell
.\run.bat tools\capture_video.py --index 1 --seconds 40 --name low_01 --out-dir data\videos
.\run.bat tools\capture_video.py --index 1 --seconds 40 --name low_02 --out-dir data\videos
.\run.bat tools\capture_video.py --index 1 --seconds 40 --name low_03 --out-dir data\videos
```

## 二、抽帧 + 预标（助手执行）

```powershell
# 每段单独抽，输出到同名段目录（split 工具按目录名分段）
.\run.bat tools\frame_extract.py --videos data\videos\low_01 --out dataset\raw_marker --every 0.8 --max-per-video 80
# 注：frame_extract 会把帧写到 <out>/<视频名>/ 下，需要再上移一层成 dataset\raw_marker\low_01\
.\run.bat tools\prelabel_marker_lowconf.py --images dataset\raw_marker --conf 0.04
```

## 三、人工改类别（阁下操作，约 20 分钟）

```powershell
.\run.bat tools\annotate.py --dir dataset\raw_marker --mode click --classes LCross,TCross,XCross
```

快捷键：`1` 强制 L、`2` 强制 T、`3` 强制 X、`0` 取消强制、`Backspace` 撤销、
右键删框、`D`/空格/`→` 下一张（自动保存）、`A`/`←` 上一张、`C` 清空、`Q` 退出。

零种子的图若确实没有交点，留空 txt（作为负样本）；但同一帧里漏掉的交点要补画。

## 四、划分 · 训练 · 评估（助手执行）

```powershell
# 按段划分：低角度留一段做 test，旧段照旧
.\run.bat tools\split_marker_dataset.py --src dataset\raw_marker --out dataset_marker ^
    --assign "low_01,low_03,marker_s03,marker_s01:marker_s04:low_02,marker_s02,marker_s05" --mode segment

# 从 v9 微调（沿用上次防 OOM 的 batch 8 / workers 0）
.\run.bat tools\train.py --data dataset_marker\marker_data.yaml --model weights\best_marker_v9.pt ^
    --epochs 40 --imgsz 640 --batch 8 --workers 0 --name marker_v10

# 帧级召回对比（真正关心的指标：单帧能稳稳抓到几个标志）
.\run.bat tools\marker_frame_recall.py --weights runs\detect\marker_v10\weights\best.pt --out outputs\marker_frame_recall_v10.csv
.\run.bat tools\marker_frame_recall.py --weights weights\best_marker_v9.pt --out outputs\marker_frame_recall_v9.csv
```

## 五、落权重 + 重解外参

```powershell
Copy-Item runs\detect\marker_v10\weights\best.pt weights\best_marker_v10.pt
.\run.bat tools\solve_extrinsics_grid.py --video data\videos\low_01 ^
    --weights weights\best_marker_v10.pt --conf 0.25 ^
    --write --out-file calibration\camera_extrinsics.yaml
```

## 验收口径

- 低角度帧在 conf 0.25 下每帧能抓到 ≥4 个标志（现在 0）
- 帧级召回（IoU 0.5）在 conf 0.25 下比 v9 高
- 解出的外参与卷尺 h=0.73 m 自洽，白线对齐 <1 px
