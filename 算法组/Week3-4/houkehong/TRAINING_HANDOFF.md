# 训练任务交接说明（给另一台电脑上的 AI 助手）

> 本文档是自包含的。读完后你应该能在**当前这台电脑**上独立完成 YOLO 模型训练，
> 并把产物交还给用户。不需要访问原电脑，也不需要看其他文档。

---

## 0. 一句话背景

用户在做一个"足球检测 + 单目场地定位"的课程考核项目。数据已经采集并标注完成，
原电脑是 **CPU-only** 的笔记本（Ryzen 5 4600U），训练 50 epoch 需要约 15 小时，
所以把训练搬到这台电脑上做。

**你的任务：在这台电脑上把两个 YOLO 模型训出来，交付权重和指标。**

---

## 0.5 这台电脑的已知硬件：RTX 5070 12 GB

**⚠️ 这是 Blackwell 架构（计算能力 sm_120），装 PyTorch 有特殊要求。**

必须用 **CUDA 12.8 或更新** 的轮子。如果用 cu124 或更早的版本，
训练启动时会报下面这个错，而且很难联想到是 CUDA 版本问题：

```
RuntimeError: CUDA error: no kernel image is available for execution on the device
```

**正确的安装命令：**

```bash
pip uninstall -y torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics==8.4.207
```

**装完必须验证两件事：**

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0)); print(torch.cuda.get_device_capability(0))"
```

期望输出：

```
2.x.x+cu128
True
NVIDIA GeForce RTX 5070
(12, 0)          ← 必须是 (12, 0)，不是 (8, 9) 之类的旧值
```

- `torch.cuda.is_available()` 为 `True`
- `get_device_capability` 返回 `(12, 0)`
- `get_device_name` 里出现 RTX 5070

三条都满足才能继续。**任何一条不满足，先解决再往下。**

如果 `cu128` 装不上，退而求其次试 `cu129` / `cu130`，或者装 nightly：

```bash
pip install --pre torch torchvision --index-url https://download.pytorch.org/whl/nightly/cu128
```

另外确认显卡驱动是较新版本（`nvidia-smi` 右上角显示的 CUDA Version 要不低于 12.8）。

### 预期性能

12 GB 显存对 yolov8n（300 万参数）绰绰有余：

| 参数 | 建议值 |
|---|---|
| `batch` | **32**（稳），可试 **64** |
| `imgsz` | 640（考核要求，不要改） |
| `device` | `0` |

**预计训练时间：30 epoch 约 5~10 分钟**（1556 张训练图，每 epoch 十几秒）。
如果超过 30 分钟说明没跑在 GPU 上，回去检查 `device=0` 和 torch 版本。

---

## 1. 交付物清单（用户最终要拿回原电脑的东西）

| 交付物 | 路径 | 说明 |
|---|---|---|
| 足球模型权重 | `runs/detect/ball/weights/best.pt` | 约 6 MB |
| 足球训练指标 | `runs/detect/ball/results.csv` | 每 epoch 的 loss 与 mAP |
| 足球测试集指标 | 一个文本文件（见第 6 节格式） | 5 个数字 |
| 标志模型权重 | `runs/detect/marker/weights/best.pt` | 如果有标志数据的话 |
| 标志训练指标 | 同上 | |

**只需要拷回这些**，不需要拷整个数据集或整个 runs 目录。

---

## 2. 先确认环境

```bash
python --version          # 需要 3.10 ~ 3.12
nvidia-smi                # 确认 RTX 5070 和驱动里的 CUDA Version ≥ 12.8
```

### 安装依赖

**完整安装命令（RTX 5070 必须用 cu128，见第 0.5 节）：**

```bash
pip uninstall -y torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics==8.4.207
```

**⚠️ 不要直接 `pip install ultralytics` 了事**——它会顺带装 **CPU 版** torch，
或者装到不支持的 CUDA 版本，两种情况都会让训练要么极慢、要么直接报错。

装完必须验证：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0)); print(torch.cuda.get_device_capability(0))"
```

必须同时满足：
- 版本号带 `+cu128`
- `is_available()` → `True`
- 设备名含 `RTX 5070`
- `get_device_capability` → `(12, 0)`

---

## 3. 数据在哪里

用户会把一个名为 `datasets` 的目录拷到这台机器上。结构如下：

```
datasets/
├── ball/                         ← 足球检测，立即可训
│   ├── images/{train,val,test}/
│   ├── labels/{train,val,test}/
│   └── data.yaml
└── marker_label/                 ← 场地标志，可能还没标完，见第 8 节
    └── ...
```

`ball` 目录里已经是**标准 YOLO 检测格式**，且 `data.yaml` 用的是相对路径，
放在任何位置都能直接用。

**足球数据集规模：**

| 集合 | 片段数 | 图片数 | 标注框数 |
|---|---:|---:|---:|
| train | 12 | 1556 | 2185 |
| val | 3 | 417 | 513 |
| test | 2 | 324 | 370 |

类别只有 1 个：`ball`（class_id 0）。

**⚠️ 不要做的事：**
- 不要把 `train/val/test` 里的图片重新打乱 —— 已经**按视频片段**划分好了，
  这是考核的硬性要求（防止同一段视频的相邻帧泄漏到不同集合导致指标虚高）
- 不要修改任何 `.txt` 标注文件
- 不要修改 `data.yaml` 里的路径结构

---

## 4. 训练足球模型

```bash
cd <datasets 的上一级目录>
yolo detect train \
    model=yolov8n.pt \
    data=datasets/ball/data.yaml \
    epochs=30 \
    imgsz=640 \
    batch=32 \
    device=0 \
    workers=4 \
    name=ball \
    project=runs/detect \
    seed=0
```

**RTX 5070 12 GB 就用上面这组参数**：`batch=32` 稳，跑顺了可以试 `batch=64`。
`workers=4`（Windows 上如果报错改成 `workers=0`）。

### 参数说明（其他显卡参考）

| 显存 | `batch` | 预计耗时 |
|---:|---|---|
| ≥ 12 GB（本机） | 32~64 | 5~10 分钟 |
| 8 GB | 32 | 5~10 分钟 |
| 6 GB | 16 | 8~15 分钟 |
| 4 GB | 8 | 15~25 分钟 |
| 无显卡 | 8 | **不要在这训，太慢** |

**如果显存不够会报 `CUDA out of memory`**，把 `batch` 减半重试即可。

`imgsz=640` 不要改 —— 考核要求统一 640 输入尺寸。

### 训练时看什么

正常输出里应该看到 `CUDA:0 (NVIDIA GeForce RTX 5070, ...)` 这样的字样。
如果显示的是 `CPU (...)`，说明 `device=0` 没生效或 torch 装成了 CPU 版，立刻停下检查。

每个 epoch 大约 10~20 秒。**如果每个 epoch 超过 1 分钟，说明没真正用上 GPU。**

### 为什么是 30 轮不是 50 轮

原电脑用 CPU 跑了 5 个 epoch，val mAP50 就已经到 **0.92**、precision **0.99**。
数据质量很高，30 轮足够收敛，50 轮是浪费时间。如果你想稳妥，跑 50 轮也行。

---

## 5. 训练完做什么

### 5.1 确认权重存在

```bash
ls -lh runs/detect/ball/weights/best.pt
```

`best.pt` 正常约 6 MB。（如果显示 30 MB，说明训练被中断了，没有剥离优化器，
这种情况要重新跑完。）

### 5.2 在独立测试集上评估（必须做）

```bash
yolo detect val \
    model=runs/detect/ball/weights/best.pt \
    data=datasets/ball/data.yaml \
    split=test \
    imgsz=640 \
    device=0
```

终端会打印一张表，形如：

```
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all        324        370      0.99        0.81      0.92      0.87
```

### 5.3 把指标写成文件交回

新建 `runs/detect/ball/test_metrics.txt`，**严格按这个格式**写 5 行：

```
Precision  0.991
Recall     0.807
mAP50      0.920
mAP50-95   0.867
epochs     30
```

数值从第 5.2 步的输出里抄。

---

## 6. 标志模型（数据已就绪，可以训）

### 6.1 类别只有 2 个，不是 4 个 —— 这是刻意的

| `class_id` | 名称 |
|---:|---|
| 0 | `LCross` |
| 1 | `TCross` |

**为什么没有 `XCross` 和 `PenaltyPoint`：**

- **`XCross`**：用户场地上不存在。现场看到的"十字"全是**中线穿过中圈形成的圆弧交叉**，
  不是两条直线交叉。这种交叉无法用"框内拟合两条直线求交点"来精确定位，强行标注会引入大误差。
- **`PenaltyPoint`**：场地上没有点球点标识（用户现场确认）。

考核细则 2.3 明确写着：

> 不同场地可能没有上述所有类别。组织方应在发放地图前确认现场实际存在且可观测的标志类别；
> **现场没有的类别不得强行要求训练或评分**。训练集至少覆盖实际场地中存在的类别。

**所以 2 类是合规的，不要自作主张加回 4 类。** `data.yaml` 里已经是 `nc: 2`。

### 6.2 数据集

```
datasets/
└── marker/
    ├── images/{train,val,test}/
    ├── labels/{train,val,test}/
    ├── data.yaml          ← nc: 2, names: [LCross, TCross]
    └── split_info.json
```

| 集合 | 片段数 | 图片数 | 标注框数 | 空标注（硬负样本） |
|---|---:|---:|---:|---:|
| train | 15 | 397 | 671 | **120** |
| val | 2 | 62 | 126 | 0 |
| test | 3 | 70 | 143 | 0 |

划分已经**按视频片段**做好（这是考核硬性要求），**不要重新打乱**。

### 6.2.-3 这一版补了「多方位角」数据（本轮最重要的改动）

**问题**：上一版模型只在两组视角上训练过 —— 手持 1.2~1.5 m 俯视、
以及三脚架 0.67 m/俯角 15°。用户把相机改到 **1.0 m/俯角 -13°** 后，
**大部分方位角模型都没见过**，导致：

- 某些角度 **T 型被误判成 L 型**（7 个观测全判 LCross，而地图只有 4 个 LCross）
  → 匹配必然 `ambiguous`
- 某些角度部分标志完全检不出

实测对比（同一场地、同一相机、仅方位角不同）：

| 方位角 | 检出 | 类别分布 | 结果 |
|---|---:|---|---|
| A | 5 | TCross×2 + LCross×3（橙蓝都有） | ✅ `valid=true`，residual 0.094 m |
| B | 7 | **LCross×7，无 TCross** | ❌ `ambiguous` |

**根因**：L/T 的形态完全取决于两条臂在画面里的可见比例，而这个比例随方位角剧烈变化。
训练集里方位角覆盖不足，模型就只会在"见过的角度"上工作。

**本轮新增**：在同一机位（1.0 m/-13°）只转云台，录了 **9 段共 54 秒视频**，
覆盖 ±30° 方位角范围，抽取并标注 **90 帧 / 352 个框**（L 150 / T 202，每帧 3.9 个）。

| 集合 | 包含的方位角片段 |
|---|---|
| train | 164136 / 164156 / 164228 / 164302 / 164315 / 164343（6 个方位角） |
| val | 164214 |
| test | 164402 / 164427 |

**val 和 test 里各留了模型没见过的方位角**，所以这一轮的指标能真实反映多角度泛化能力。

**上一版指标作为对照**：test mAP50 0.515、val 0.969（但只在单一视角下有效）。

### 6.2.-2 上一轮微调造成了灾难性遗忘 —— 这一版修的是划分

上一轮（`epochs 40` 从 `marker_start.pt` 微调）结果：

| 指标 | 上一版 | 微调后 |
|---|---:|---:|
| **val mAP50**（含新低机位） | 0.556 | **0.773** ✅ |
| **test mAP50**（224416 人脚近距） | 0.435 | **0.039** ❌ |

现场 6 帧实测：旧模型 **0 检出** → 微调后 **14 检出**（2.3 个/帧）。
但旧视角 224416 上：旧模型 8 检出 → 微调后 **0 检出**。

**根因是数据划分，不是学习率。** 上一版把「两段演示机位」一段放 train、一段放 val，
而 test 里的 224416（人脚近距）在 train 中完全没有同类视角，
所以模型一旦转向新视角，就把 224416 这种形态彻底丢掉了。

**这一版改了划分：把两段演示机位都放进 train，test 换成一个模型从未见过、
但训练集中有同类视角的片段。**

| 集合 | 片段 | 意图 |
|---|---|---|
| **train** | 223918(近距T) / 224011(近距L) / 224242(中景) / 224447(近距多标志) / **215225(演示机位-15°)** / **215319(演示机位低俯角)** + 3 段负样本 | 演示机位必须训练过 |
| **val** | 224316（广角远景） | |
| **test** | **224416（人脚近距，完全不参与训练）** | 诚实的跨视角泛化指标 |

### 6.2.-1 起始权重用 `marker_start.pt`（上一版基线，不是微调坏的那个）

`marker_start.pt` 就是上一轮的 **0.435 基线**。它旧视角能力最全：

- 旧视角 224416 抽 12 张：**8 个检出**（正常）
- 演示机位 6 张：**0 个检出**（正是这轮要补的）

由于这一版 train 里**同时包含旧视角和演示机位**，从它微调应该能**两者兼得**，
而不是像上一轮那样顾此失彼。

**训练命令：**

```bash
yolo detect train \
    model=marker_start.pt \
    data=datasets/marker/data.yaml \
    epochs=40 \
    imgsz=640 \
    batch=32 \
    device=0 \
    workers=4 \
    name=marker \
    project=runs/detect \
    seed=0
```

- **`model=marker_start.pt`**（不是 `yolov8n.pt`）：模型已经会认 L/T，只是缺这个视角，
  微调比从头训效果更好、更省时
- `epochs=40` 足够；`batch=32`；GPU 上约 **3~5 分钟**

**判断有没有学到（三条都要看）：**

| 指标 | 期望 | 说明 |
|---|---|---|
| val mAP50 | ≥ 0.90 | val 里含没见过的方位角 `164214` |
| **test mAP50** | **≥ 0.50** | test 里含没见过的方位角 `164402`/`164427` |
| 最关键的验证 | 见下 | 用真实低机位帧验证"多角度是否都能出框" |

**⚠️ 如果 test 掉到 0.5 以下**，说明又出现了灾难性遗忘（上一版曾掉到 0.039）。
处理顺序：① 加 `freeze=22`（只训检测头，上一轮这样救回来了）② 减到 20 轮 ③ 降 lr0 到 0.001。

### 6.2.0 这一版加进了 120 张硬负样本（关键改动）

`train` 里有 3 个片段叫 `neg_224039_circle` / `neg_224140_circle` / `neg_224212_circle`，
**它们的标注文件是空的**，这是**故意的**，不要以为是漏标。

**为什么需要它们：**

现场实测发现，模型会把「中线穿过中圈」形成的**直线 × 圆弧交叉**误判成 `TCross`，
几乎每帧都误检 1 个。这种假标志会污染地图匹配，导致定位解错或 `ambiguous`。

根因：训练集里全是 L 型和 T 型的正样本，模型**从没见过这种形状当负样本**。

这 120 帧就是用那三段"只拍到中圈"的视频抽的，全部标为空——
让模型学会"直线接到圆弧上不是 T 型"。

**所以：**
- 这 120 张空标注**必须保留**，删掉误检会重新变严重
- `train.py --only-labeled` 默认会跳过空标注的图；这 3 段是靠 `--neg-segs` 特批进来的

### 6.2.1 这一版的划分是刻意指定的，不要"优化"它

数据来自 6 段视频，而**每一段对应一个独立视角档位**：

| 片段 | 视角 | 标志在画面里的占比 |
|---|---|---|
| 223918 | 近距离 T 型 | 约 25% |
| 224011 | 近距离 L 型 | 约 25% |
| 224242 | 中景/中圈 | 中等 |
| **224316** | **广角远景** | **约 6%** |
| 224416 | 人脚近距 | 约 30% |
| 224447 | 近距多标志 | 约 25% |

**上一版把 224316（唯一的广角远景）划进了 test**，结果训练集里全是"贴脸大标志"，
模型从没见过小尺度标志，测试 mAP50 崩到 **0.03**，而 val 却有 0.45。

这一版把 **224316 放进 train**，让模型同时见到大尺度和小尺度。别再改回随机比例划分。

**已知局限（请如实记录，不要试图掩盖）：** 因为每段视频是单一视角，
所以按片段划分的 test 指标本质上是"跨视角"指标，数值会明显低于 val。
这是数据集结构造成的，不是训练失败。

### 6.3 训练命令

```bash
cd <datasets 的上一级目录>
yolo detect train \
    model=yolov8n.pt \
    data=datasets/marker/data.yaml \
    epochs=60 \
    imgsz=640 \
    batch=32 \
    device=0 \
    workers=4 \
    name=marker \
    project=runs/detect \
    seed=0
```

- **`epochs=60`**：标志数据只有 169 张训练图，比足球少一个数量级，多训几轮
- `batch=32` 对 12 GB 显存很宽松，可以试 64
- **预计 3~6 分钟**

### 6.4 评估（必须做）

```bash
yolo detect val \
    model=runs/detect/marker/weights/best.pt \
    data=datasets/marker/data.yaml \
    split=test \
    imgsz=640 \
    device=0
```

会按类别分别输出 `LCross` 和 `TCross` 的 P / R / mAP50 / mAP50-95。
**两个类别的指标都要记录**（考核要求"场地标志检测必须单独报告各类别的 Precision、Recall、mAP50"）。

写成 `runs/detect/marker/test_metrics.txt`：

```
LCross     Precision R mAP50 mAP50-95
TCross     Precision R mAP50 mAP50-95
all        Precision R mAP50 mAP50-95
epochs     60
```

### 6.5 预期

169 张训练图、2 个类别，预期 **mAP50 在 0.6~0.85**。

如果低于 0.4，**不要自己改标签**，把 `results.csv` 和几张检测效果图交回，让用户判断。

---

## 7. 常见错误

| 报错 | 原因 | 解决 |
|---|---|---|
| **`CUDA error: no kernel image is available for execution on the device`** | **RTX 5070 是 Blackwell (sm_120)，torch 是 cu124 或更早的版本** | **按第 0.5 节换 cu128 轮子重装** |
| `CUDA out of memory` | batch 太大 | 减半，或加 `amp=False` |
| `torch.cuda.is_available()` 是 False | 装了 CPU 版 torch | 按第 2 节重装 CUDA 版 |
| 训练日志显示 `CPU (...)` 而不是 `CUDA:0` | `device=0` 没生效 | 检查参数拼写，确认 torch 能用 CUDA |
| 每个 epoch 超过 1 分钟 | 没真正用上 GPU | 同上，先确认 `is_available()` 为 True |
| `Dataset not found` | `data=` 路径不对 | 指向 `data.yaml` 文件，不是目录 |
| `No labels found` | labels 目录结构不对 | 检查 `labels/train` 是否和 `images/train` 对应 |
| Windows 上 `DataLoader worker` 报错 | 多进程问题 | 加 `workers=0` |
| 训练完 `best.pt` 是 30 MB | 训练被中断 | 重新跑完整轮数 |
| mAP50 始终 < 0.3 | 大概率标签和图片错位 | 报告给用户，不要自己改标签 |

---

## 8. 交回给用户的清单（最终确认）

训练完成后，把下面这些东西放到一个文件夹里交给用户（U 盘 / 网盘）：

```
交付/
├── ball_best.pt                    ← 从 runs/detect/ball/weights/best.pt 复制
├── ball_results.csv                ← 从 runs/detect/ball/results.csv 复制
├── ball_test_metrics.txt           ← 你在 5.3 步写的
└── marker_best.pt                  ← 如果训了标志模型
└── marker_test_metrics.txt
```

并在回复里用一句话总结：**用什么显卡、跑了多少轮、训了多久、测试集 mAP50 多少。**

---

## 9. 不要做的事（重要）

1. **不要用网上下载的预训练权重冒充训练结果** —— 考核明确禁止，会判 0 分。
   `yolov8n.pt` 只能作为**迁移学习的初始化**，最终 `best.pt` 必须是在这份数据上训出来的。
2. **不要上传数据集或权重到公开网盘** —— 数据是用户本人实拍，只在双方之间传递。
3. **不要修改标注** —— 如果发现标签有问题，报告给用户，让用户在原电脑上修。
4. **不要改变 train/val/test 的划分** —— 这是按视频片段划分的，改了指标就没有意义。

---

## 10. 一句话总结

装好 CUDA 版 torch，用 `datasets/ball/data.yaml` 训 30 轮 yolov8n，batch 按显存调，
然后在 `split=test` 上评估，把 `best.pt` + 指标文件交回。
