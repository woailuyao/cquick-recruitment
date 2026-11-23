# 数据集说明（dataset_summary.md）

> 按细则要求：GitHub 只提交本文件与轻量配置，**不上传原始视频、抽帧图片、标注图片、
> 训练曲线图和 `runs/` 目录**。原始素材全部本地保存在 `D:\CQU\CQuick\week3`，备查。

## 1. 采集概况

| 项 | 值 |
|---|---|
| 采集日期 | 2026-11-12（实验室 C0 场地） |
| 摄像头 | 单目 USB RGB，1280×720 MJPG，实测 30 fps，固定焦距 / 固定对焦 |
| 原始视频 | 19 段（`raw_videos/`），其中任务一使用 7 个独立片段 |
| 场地实测 | 8.35 m × 6.00 m（含白线），`config/field_geometry.yaml` |
| 坐标系 | C0：原点=场地中心，+X 指向唯一球门，+Y 面向 +X 时的左侧，单位米 |

## 2. 任务一：足球检测（ball，单类）

| 项 | 值 |
|---|---|
| 抽帧总数 | **282 张**（`frames_pool/`） |
| 已剔除 | 100 张宿舍橙子调试帧 + 3 个误标 txt，归档在 `frames_pool_dorm_rejected/`，未删除 |
| 标注实例 | **214 个 ball 框**；68 张空标签（遮挡 / 关灯 / 空场 / 无球） |
| 标注工具 | 零安装 `tools/annotate_cv2.py`（仅 OpenCV，不联网、不装包） |
| 格式 | YOLO txt，归一化 `cls cx cy w h`，图片与标签同名一一对应 |

**划分方式**：按**独立视频片段**分组做 70/20/10（`split_dataset.py`，seed=0），
而不是按帧随机划分 —— 同一段视频的帧不会跨 train/val/test，避免时序泄漏。
纯空场片段强制划入 train，保证 val/test 各至少含 1 个含球片段。

| split | 图片 | 含球图片 | 备注 |
|---|---|---|---|
| train | 215 | 173 | 含 42 张空标签 |
| val | 35 | 10 | `ball_run07`（开关灯片段） |
| test | 32 | 31 | `ball_run05`（**独立远距小球片段**，尺度外 OOD） |

**正式训练数据 `dataset_aug/`**：train 在 215 张基础上补 169 张
`tools/make_small_ball_aug.py` 生成的伪远距小球增强图（整图缩小 + 同图模糊背景，
无色差拼接），共 **384 张**；val/test 原样复制，指标可与 v1 直接对比。
配置 `config/data_aug.yaml`。增强图**不是真实拍摄**，报告中已如实说明。

**v4 实验数据 `dataset_aug2/`**（2026-11-20 新增）：在 `dataset_aug/` 基础上
追加 **40 张无球负样本**（`frames_pool_markers/` 中 11 个无球视频段等间隔抽取，
空标签），用于压制地球仪 / 栏杆 / 设备箱扣等类球干扰误检；train 共 424 张。
配置 `config/data_aug2.yaml`。详见 `logs/negative_samples_20261120.txt`。
**结果**：v4（seed=0）test mAP50 0.995、误检 0 / 漏检 1，已定为正式权重
`best_train_v4.pt`；但双 seed 对照显示小数据集训练方差极大
（v3 换 seed=1 从 0.613 掉到 0.136），报告须连同方差一起写，
见 `docs/训练报告_任务一.md` 第 8 节 / `logs/train_runs.csv`。

> 类别分布：`ball` 单类，无类别不平衡之外的多类问题；主要难点是**尺度跨度**
> （train 框宽中位 169.6 px、最大 742 px 近距特写；test 中位仅 82.5 px）。

## 3. 任务二：场地标志（4 类）

| 项 | 值 |
|---|---|
| 抽帧 | **210 张**（`frames_pool_markers/`）：11-12 现场 175 张 + 11-20 C0 现场手持走位补录 `markers_sweep_20261120_175438.mp4` 抽出 35 张；来自 12 个视频段 |
| 类别 | `L_cross`(0) / `T_cross`(1) / `X_cross`(2) / `penalty_point`(3)，共 15 个场地标志 |
| 标注状态 | ✅ **已全部人工标注完成**（2026-11-21）：210 张全有同名 txt，121 张含标志 / 89 张空负样本，共 **301 个框** |
| 四类框数 | `L_cross` 158 / `T_cross` 61 / `X_cross` 54 / `penalty_point` 28 |
| 自检 | `check_labels.py` **0 格式错误、0 缺标签**（`logs/label_check_20261121_225233.txt`） |
| 工具 | `tools/annotate_cv2.py --classes L_cross,T_cross,X_cross,penalty_point`；首轮曾出现「只标 L、其余三类零样本」，用 `--only-labeled` 复核模式补标修正，未重拍、未伪造框 |

**划分方式**：按**独立视频段** 70/20/10、**`seed=102`**（不是 seed=0）。
改 seed 的原因如实记录：seed=0 时 test 恰好只落到 `marker_run05`（34 张里 31 张空图、
仅 3 张有框、`T_cross` 无真值），四类 mAP 无法有效计算；在 200 个候选 seed 中按
「val/test 均四类齐全 + 各有 ≥8 张含标志图」筛出 47 个，选 seed=102。
**筛选依据是评估集覆盖度，不是挑好看的指标**（当时尚未训练）。
最终 train 130 / val 52 / test 28（四类框 L20/T15/X15/P7），无视频组跨 split 泄漏。

| 项 | 值 |
|---|---|
| 训练入口 | `config/marker.yaml`；基线 `yolov8n.pt / imgsz=640 / epochs=50 / device=0`（训练 seed=0） |
| 权重 | `runs/detect/markers/weights/best.pt`（根目录副本 `best_markers.pt`），**按细则不上传 GitHub** |
| 独立 test 真实结果 | **P 0.940 / R 0.050 / mAP50 0.113 / mAP50-95 0.034** |
| 分类别 AP50 | `L_cross` 0.169 / `T_cross` 0.024 / `X_cross` 0.259 / `penalty_point` **0.000** |

⚠️ **指标很差，如实记录不修饰**：高 P 极低 R = 模型只敢报极少数高置信框、绝大多数交点漏检；
点球点 AP 为 0（训练集仅 17 张含它、28 个框）。根因是 301 个框分散在 4 类、
每类样本极少 + 交点框只有 8~25 px 的小目标 + 50 epoch 基线未做增强。
**不据此声称「四类标志识别可用」或「实时定位链路已跑通」**。
记录见 `logs/train_runs.csv`、`docs/train_args_markers.md`、`docs/README_提交版.md` 任务二节。

类别编号一经确定不得插入新类；与 `task2/` 包的 `field_by_class` 一致。

## 4. 位姿真值

`logs/field_pose_samples.csv`：**4 行卷尺实测参考真值** —— `pose_A` / `pose_B` /
`pose_C` / `pose_D2`，字段 `视频文件, x, y, yaw_deg, 相机高度m, 备注`。
前三组由现场参照物（水瓶）地脚像素 + 透视联立反解，备注里记录了解算方式与
已知偏差（如「远处标志预测偏左 ~150px，fx 呼吸效应待评估」）；
`pose_D2` 为 2026-11-20 C0 现场补测：卷尺量光心投影 (x,y) 与光轴落点、
O→P 连线算 yaw，白盒对中偏差 9 px（yaw 偏差 <0.4°）。

误差结果（**离线人工目视点选 + PnP**，非实时自动定位）：
`pose_B` 32.1 cm / 7.3° / 3.4 cm、`pose_C` 9.1 cm / 4.5° / 4.8 cm、
`pose_D2` 13.3 cm / 2.2° / 3.5 cm 共 **3 组有效**；
`pose_A` 仅 2 个可见交点，判为 `valid=FALSE` / `INSUFFICIENT_MARKERS` 无效状态案例。
详见 `docs/任务二误差表.md`。
**该真值只用于误差表对比打分，绝不参与定位解算或点名分配。**

## 5. 复现命令

```bash
python tools/split_dataset.py --dry-run          # 查看划分表
python tools/make_small_ball_aug.py              # 生成 dataset_aug
python tools/make_negative_samples.py            # 生成 dataset_aug2
python tools/train_yolo.py --data config/data_aug.yaml --name train_v3
python tools/eval_cases.py --model best_train_v4.pt
```

## 6. 已知限制（如实记录）

- test 的球框面积仅 train 的 22%，属尺度外 OOD；提高 imgsz 无法解决相对尺度差。
- v3 误检从 v1 的 4 涨到 35（含 0.60 高置信误检），v4 负样本实验即为针对此问题。
- val 仅 10 个球且多在画面顶边被裁，best.pt 选择不稳；未拿 test 挑 checkpoint。
- 100 张宿舍橙子调试帧 + 3 个误标 txt 已归档（`frames_pool_dorm_rejected/`）而非删除，不参与任何训练/评估。
- 四类标志模型 301 框 / 4 类，每类样本极少，test R 仅 0.050、点球点 AP50 = 0，
  **实时帧大概率仍频繁漏检**；本轮未用它补录有效位姿，15:10 录屏 `valid=FALSE` 结论不变。
  「实时相机 → 自动标志检测 → 位姿」链路未跑通，README 中如实分开陈述。
