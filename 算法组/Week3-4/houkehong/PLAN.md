# 执行规划（2026-11-19 更新）

> 足球模型已训练完成并验收通过。本文件只保留"现在该做什么"。

---

## 一、当前状态

### ✅ 已完成

| 项目 | 结果 |
|---|---|
| 环境（Python/PyTorch/Ultralytics/WSL2/ROS2） | 全部就绪 |
| 相机 | 外接 USB，MSMF 后端 30 fps，1280×720 |
| **内参标定** | ✅ RMS 0.2126 px，fx=1567.7 / fy=1567.4 / cx=666.3 / cy=356.5 |
| 畸变 | 轻微（全画面最大位移 6.15 px） |
| 足球数据 | 17 段 / 2297 张 / 标注完成 |
| **足球模型** | ✅ **训练完成**：test mAP50 **0.906** / P 0.981 / R 0.811（RTX 5070，30 epoch） |
| **实时链路** | ✅ 真实模型 + 真实相机跑通，**14.4 fps @416** |
| 标志数据 | 9 段 / 2400 帧 → 403 张**待标注** |
| 定位算法链路 | 合成真值误差 0.03 m / 0.2° |
| ROS2 话题 | 3 个话题实测通过 |
| 左右分屏可视化 | 完成 |

### ❌ 未完成

| 项目 | 缺什么 | 阻塞什么 |
|---|---|---|
| **外参** | 相机未固定到三脚架，仍是占位值 0.55 m / -22° | 任务 2 的地面投影（**当前唯一硬阻塞**） |
| **C0 现场测试** | 未做 | 任务 2 验收 |
| 录屏 / B站 / GitHub PR | 未做 | 两个任务的材料完整性 |
| 标志地图覆盖范围（可选） | 8 个标志都挤在球门区，建议再补 2~4 个外圈标志 | 只是扩大可定位区域，不阻塞 |

> 两个检测模型都完成，场地地图已实测生成并通过闭环验证。
> **现在只差外参**——量完就能出真实位姿。

### 📌 实测地图（2026-11-19 生成）

坐标系按组长定义：原点 = 中圈圆心，+X 指向球门，+Y = 面向球门的左侧。

| 文件 | 说明 |
|---|---|
| `measurements.txt` | 8 个标志的实测数据（4 L + 4 T） |
| `config/c0_field.yaml` | 生成的地图坐标（含点球点 (0,0)） |
| `config/c0_field_geometry.yaml` | 原点/轴向/范围声明 |

**闭环验证结果**（真实地图 + 真实内参 + 假定外参 h=1.30/pitch=-20°）：

| 像素噪声 | 位置误差 | 朝向误差 | 成功率 |
|---|---:|---:|---:|
| 2 px | 3~8 cm | 0.2~0.3° | 100% |
| 5 px | 6~12 cm | 0.3~0.7° | 93~100% |

**工作区域**：相机 X ≤ -0.5 m（中圈圆心后方至少半米，面朝球门），最远测到 -7 m 都稳定。
站到中圈圆心上（X=0）只能看见 2 个左右对称的 T 型标志，会报 `ambiguous`。

---

## 二、按优先级排序的行动

### P1 · 立刻做（10~15 分钟）

**复核足球的 157 张空标注**

```powershell
cd D:\vision_ws\k1_soccer_vision
.\.venv\Scripts\python.exe scripts\label_tool.py --src dataset_raw --task ball --only-empty
```

- 球不在画面 → 直接按 `→` 翻过（空标注是正确的负样本）
- 球在但没框 → 拖一个框

**为什么先做这个**：如果这 157 张是"有球但漏标"，模型会被教成"这种场景没有球"，
正好学反。它们集中在贴白线、遮挡、远处小目标这些最难的场景。

---

### P2 · 立刻启动（后台 2 小时，不用盯着）

**训练足球模型**

```powershell
.\.venv\Scripts\python.exe scripts\train.py --src dataset_raw --task ball --epochs 50 --batch 8 --workers 0
```

- CPU 训练约 1.5~2.5 小时
- 自动按片段划分 train/val/test，并在独立测试集上按类别出指标
- 训练期间占满 6 核，电脑会卡，别同时干重活

---

### P3 · 训练期间去场地量尺寸（不需要电脑，30 分钟）

**验证官方 field.yaml 能不能直接用** —— 决定任务 2 成败，别拖到最后。

带卷尺量这三个距离：

| 量什么 | 官方值 | 允许误差 |
|---|---:|---|
| 场地两端 L 型角点之间 | 8.950 m | ±5 cm |
| 两条边线之间（上下对应两点） | 6.940 m | ±5 cm |
| 两个 X 型（中线交叉）之间 | 2.940 m | ±5 cm |

**全部在误差内** → 直接用官方坐标，`field_geometry_official.yaml` 改 `verified: true`
**超差** → 必须按实测自己定义 `field.yaml`，我帮算坐标

同时记录：现场**实际存在哪些标志类别**（有没有 X 型？有没有点球点？）、
三脚架打算用的**高度和俯角**。

---

### P4 · 看足球指标（训练结束后，10 分钟）

```powershell
type runs\ball\test_metrics.txt
```

| mAP50 | 判断 | 下一步 |
|---|---|---|
| > 0.8 | 很好 | 直接进 P5 |
| 0.6~0.8 | 可用 | 进 P5，补标困难样本还能提 |
| < 0.6 | 有问题 | 回头查标签（大概率空标注处理不当） |

---

### P5 · 标标志种子集（40 分钟）

```powershell
.\.venv\Scripts\python.exe scripts\label_tool.py --src dataset_marker_label --task marker
```

- 类别键：**`1`=L 型（2 条臂）`2`=T 型（3 条臂）`3`=X 型（4 条臂）`4`=点球点**
- 框不用精确，后面 `landmarks.py` 会在框内做 Hough 细化
- **标 80 张就够**，按 `q` 退出

训练种子模型：

```powershell
.\.venv\Scripts\python.exe scripts\train.py --src dataset_marker_label --task marker --only-labeled --epochs 40 --batch 8 --workers 0 --name marker_seed
```

**`--only-labeled` 必须加**：不加的话 323 张未标注图会被当成负样本，模型训废。

---

### P6 · 用种子模型标剩下 323 张（约 1 小时）

```powershell
.\.venv\Scripts\python.exe scripts\prelabel.py --src dataset_marker_label --task marker --weights runs\marker_seed\weights\best.pt --conf 0.15
.\.venv\Scripts\python.exe scripts\label_tool.py --src dataset_marker_label --task marker
```

修正远比从零画快。修完训最终模型：

```powershell
.\.venv\Scripts\python.exe scripts\train.py --src dataset_marker_label --task marker --epochs 50 --batch 8 --workers 0
```

---

### P7 · 固定相机 + 外参标定（30 分钟）

1. 相机固定到三脚架，**高度和俯角定死后贴胶带做记号**
2. 量高度 h（卷尺）；在地面距离 D 处放个标记，读出它在画面里的纵坐标 v
3. 反解俯角：`θ = atan(h/D) − atan((v − cy)/fy)`
4. 填进 `config/camera.yaml` 的 `extrinsics`

这一步的工具我还没写，到时候补。

---

### P8 · 整链联调 + C0 三点评测（1 小时）

```powershell
.\.venv\Scripts\python.exe run.py live --camera 1 `
    --ball-weights runs\ball\weights\best.pt `
    --marker-weights runs\marker\weights\best.pt --imgsz 416
```

- 至少 3 个不同位置/朝向
- 记录每个点的估计位姿 vs 组长给参考值
- 验证标志不足时正确输出 `valid=false`

---

### P9 · 收尾（2 小时）

1. 录屏（开头必须 `echo "姓名: xxx"; date`）
2. 上传 B 站
3. 填 README
4. GitHub Fork → PR
5. 加分项（有余力）：ONNX 对比 +3、RANSAC 匹配 +2、ROS2 launch +2

---

## 三、时间预算

| 阶段 | 耗时 | 需要电脑 |
|---|---:|---|
| P1 复核空标注 | 15 min | ✅ |
| P2 足球训练 | 1.5~2.5 h | ✅ 后台 |
| P3 场地量尺寸 | 30 min | ❌ |
| P4 看指标 | 10 min | ✅ |
| P5 标标志种子 | 40 min | ✅ |
| P6 标志预标+修正+训练 | 1.5 h | ✅ |
| P7 固定+外参 | 30 min | ✅ |
| P8 联调+现场测试 | 1 h | ✅ |
| P9 收尾 | 2 h | ✅ |
| **合计** | **约 8~9 小时** | 其中 4~5 h 是等训练 |

---

## 四、风险

| 风险 | 影响 | 应对 |
|---|---|---|
| 157 张空标注大量是漏标 | 模型在困难场景学反 | P1 必须先做 |
| 官方地图与实地不符 | 任务 2 定位整体错 | P3 卷尺验证 |
| 现场只有 L/T 两类标志 | X 型训不出来 | 按实际类别调整 `MARKER_ORDER` |
| CPU 训练太慢 | 赶进度 | 数据集已够，别加大；`imgsz` 保持 640 |
| 8 GB 内存 | 训练 OOM | `workers=0`、关浏览器 |
