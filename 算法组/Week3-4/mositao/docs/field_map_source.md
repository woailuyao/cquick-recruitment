# 官方 field.yaml 的来源与核对记录

> 本文件记录：从官网**原样取得**的场地地图、校验值、逐点坐标，以及「能确定什么 / 还不能确定什么」。
> 目录里的 `config/field.yaml` 是官方字节副本，**未改动任何坐标**（细则 §2.11）。

## 一、来源

| 项 | 内容 |
| --- | --- |
| URL | `https://raw.githubusercontent.com/BoosterRobotics/robocup_demo/main/src/vision/config/field.yaml` |
| 备用镜像 | `https://cdn.jsdelivr.net/gh/BoosterRobotics/robocup_demo@main/src/vision/config/field.yaml`（两份内容一致，均 200） |
| 仓库 | BoosterRobotics/robocup_demo，分支 `main`，路径 `src/vision/config/` |
| 取得时间 | 2026-11-15 19:10 |
| 字节数 | 369 B |
| SHA256 | `1d65e27b1491712392d43b8b8040224d5561125f9419b505081143a88183cf73` |

## 二、官方原文（14 个特征点，原样）

```yaml
TCross1: [ 5.955,  2.930]
TCross2: [ 4.945,  2.930]
TCross3: [ 1.000,  2.930]
TCross4: [ 0.000,  2.930]
TCross5: [ 7.465, -4.010]
TCross6: [-1.485, -4.010]
LCross1: [ 7.465,  2.930]
LCross2: [-1.485,  2.930]
LCross3: [ 4.945,  2.020]
LCross4: [ 1.000,  2.020]
LCross5: [ 5.955,  0.000]
XCross1: [ 4.505, -4.010]
XCross2: [ 1.565, -4.010]
PenalityPoint: [ 2.965,  0.885]
```

类别分布：`TCross=6, LCross=5, XCross=2, PenaltyPoint=1`，合计 14。
点球点键名是官方拼写 `PenalityPoint`，代码里已映射为模型类别 `PenaltyPoint`。

## 三、从坐标直接算出的事实

| 项 | 数值 |
| --- | --- |
| x 跨度 | `[-1.485, 7.465]` → **8.95 m** |
| y 跨度 | `[-4.010, 2.930]` → **6.94 m** |
| 跨度中心 | `(2.990, -0.540)` |

三点可以直接读出来：

1. **单点球点**：`PenalityPoint` 只有 1 个，说明这份地图对应的场地要么是单球门，要么只测了一端。
2. **原点不在跨度中心**：若原点在场地中心，跨度中心会落在 (0, 0) 附近，实际是 (2.990, −0.540)，差了一整个半场。
3. **不是 Booster 定位器自生成的那套特征**。`locator.cpp` 的 `calcFieldMarkers()` 按 `FieldDimensions` 生成 **26 个**特征，其中 `X` 在 `(0, ±circleRadius)`（竖直分布）、`P` 在 `(±(length/2−penaltyDist), 0)`（**两个**、都在 y=0）。而本文件只有 14 个点、只有 1 个 P，两个 `X` 在 `y = −4.010` 这条线上水平分开 2.94 m。**任何刚体变换都变不出这个差异**，所以 `field.yaml` 是现场实测得到的**特征点子集**，不是构造出来的理想场地。

## 四、还不能确定什么

> **已部分解决**：下表四项已于 2026-11-15 19:30 由 Booster 源码反推得出，见本文 §七。
> 反推结论仍待组长确认，但已不再是无解状态。

| 未知项 | 为什么 | 结论 |
| --- | --- | --- |
| 原点定义 | 官方文件未声明，且跨度中心不是 (0,0) | 必须由 `field_geometry.yaml` 给出 |
| `+X` / `+Y` 轴向 | 同上 | 同上 |
| 场地长度 / 宽度 | 跨度含球门线与边线，但标志不覆盖完整矩形 | 同上 |
| 单位 | 按米理解与 8.95×6.94 量级自洽 | 大概率 m，待确认 |

细则 §2.2 原文即如此：「该文件本身未必包含完整场地尺寸、球门尺寸、所有线条和地图坐标系定义，因此组织方必须配套发布 `field_geometry.yaml`。」

**已确认缺失**：Booster 仓库只有 `field.yaml` / `vision.yaml` / `config.yaml` 三个配置文件，没有 `field_geometry.yaml`；CQuick 组织方仓库 `woailuyao/cquick-recruitment` 的 `算法组/Week3-4/` 下只有 `README.md`，也没有发布。所以这份 `field_geometry.yaml` 仍在组长手上。

## 五、工程侧已完成的适配

| 动作 | 结果 |
| --- | --- |
| `config/field.yaml` 换成官方字节 | SHA256 `1d65e27b…cf73`，369 B，坐标未动 |
| 旧占位地图留底 | `config/field.placeholder.yaml`（14×9 m、原点居中，尺度与官方不符，仅留档） |
| `src/loc/field_map.py` 解析器 | 新增扁平写法 `Key: [x, y]` 分支 + 键名去尾部序号，官方格式可解析 |
| 解析自检 | 14 点全部读出，类别分布与官方一致 |
| 流水线自检 | `src/sim/test_pipeline.py` 八项全通过（往返 2.758e-14 m、外参 0.499 m/21.99°） |

## 六、待办

1. 向组长索取 `field_geometry.yaml`（原点、轴向、单位、尺寸、地图版本）；到手后整体替换 `config/field_geometry.yaml` 并重跑 `build_field_map.py`。
2. 请组长确认 C0 现场实际存在的标志类别（细则 §2.3：现场没有的类别不得强行训练或评分）。官方这份只有 T/L/X/P 四类，与建议类别表一致。
3. 官方地图与 C0 是否一致，由组长核对并发布结论（细则 §2.11）；我们只记录，不改坐标。

## 七、坐标系反推（2026-11-15 补）

### 7.1 找到的「配套」文件

阁下说得对，Booster 确实有配套几何，只是**不叫 `field_geometry.yaml`**，而在源码里：

| 文件 | 内容 |
| --- | --- |
| `src/brain/include/types.h` | `struct FieldDimensions` + 三套常量 |
| `src/brain/src/brain_config.cpp` | 读 `config.yaml` 的 `field_type` 选常量、生成地图线 |
| `src/brain/config/config.yaml` | `field_type: "adult_size"` |
| `src/brain/src/locator.cpp` | `calcFieldMarkers()` 用上述尺寸生成 26 个理想特征 |

`types.h` 原文：

```cpp
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.6, 2.6, 2, 2.25, 6.9, 0.75, 3.9};
```

字段顺序：`length, width, penaltyDist, goalWidth, circleRadius, penaltyAreaLength, penaltyAreaWidth, goalAreaLength, goalAreaWidth`。

### 7.2 反推方法

用 `calcFieldMarkers()` 的公式按 **kid_size / adult_size** 各生成 26 个理想特征，再对官方 14 点搜刚体变换（0°/90°/180°/270° 四档 + 用两个 XCross 定锚求平移），看每个观测点能否落到**同类别**的理想点上。

### 7.3 结果：14/14 全部命中 adult_size

| 场地模型 | 旋转 | 最差残差 | 平均残差 | <0.15 m 命中 |
| --- | ---: | ---: | ---: | ---: |
| kid_size | 任意 | 2.875 m | 1.865 m | 0/14 |
| **adult_size** | **90°** | **0.108 m** | **0.067 m** | **14/14** |

逐点对应（残差为拟合后的欧氏距离）：

| 观测点 | 残差 m | → 场地坐标（Booster 中心系） |
| --- | ---: | --- |
| TCross1 | 0.100 | (7.000, −3.000) 球门线 × 罚球区侧线 |
| TCross2 | 0.108 | (7.000, −2.000) 球门线 × 球门区侧线 |
| TCross3 | 0.069 | (7.000, +2.000) |
| TCross4 | 0.069 | (7.000, +3.000) |
| TCross5 | 0.070 | (0.000, −4.500) 中线 × 边线 |
| TCross6 | 0.020 | (0.000, +4.500) |
| LCross1 | 0.092 | (7.000, −4.500) 场地角 |
| LCross2 | 0.063 | (7.000, +4.500) 场地角 |
| LCross3 | 0.095 | (6.000, −2.000) 球门区前沿角 |
| LCross4 | 0.046 | (6.000, +2.000) |
| LCross5 | 0.081 | (4.000, −3.000) 罚球区前沿角 |
| XCross1 | 0.030 | (0.000, −1.500) 中圈 × 中线 |
| XCross2 | 0.030 | (0.000, +1.500) |
| PenalityPoint | 0.070 | (4.900, +0.000) 点球点 |

残差 max 0.108 m / mean 0.067 m / RMS 0.072 m，量级与「拿卷尺在场地上量线交点」的误差吻合，说明官方 file 是**实测**而非生成。

### 7.4 由此确定的坐标变换

官方 `field.yaml` 的数值**不是**场地中心系，它相对场地中心旋转 90° 并平移：

```
x_field =  file_y + 4.010
y_field = -file_x + 3.035
```

即场地中心落在 file 坐标 `(3.035, −4.010)`；`file +X` 对应场地 `−Y`，`file +Y` 对应场地 `+X`。

### 7.5 两个必须记住的结论

1. **场地是 adult_size：14 m × 9 m**（中圈半径 1.5 m、点球点距球门线 2.1 m、球门宽 2.6 m）。
   这同时解释了之前「宽大于长」的怪相：官方地图的包围盒 8.95 × 6.94 m **只是半个场地**。
2. **官方地图只覆盖半个场地**：14 个点全部落在场地 `x ∈ [0, 7]`（中线到 +X 端球门线）。
   另半场没有任何特征点。若 C0 是全场，另一半无法定位——这一点必须让组长知道。

### 7.6 点球点的位置

`PenalityPoint → (4.9, 0.0)`，即距 +X 端球门线正好 2.1 m（= `FD_ADULTSIZE.penaltyDist`），横向 `y = 0` 居中。

所以：点球点确实压在**场地中轴**上（横向居中），但不在场地中心，沿长轴离中心 4.9 m。

### 7.7 复现方式

拟合脚本未纳入交付（属一次性分析）。若要复现：按 §7.2 用 `FD_ADULTSIZE` 生成 26 点，取 `θ = 90°`、平移 `M = (3.035, −4.010)`，逐点找同类别最近邻即可。
