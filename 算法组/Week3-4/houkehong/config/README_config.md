# 配置文件说明

| 文件 | 内容 | 用途 |
|---|---|---|
| `field.yaml` | **C0 实测场地地图**（9 个特征，`map_version: C0 measured (personal)`） | 任务 2 实际使用 |
| `field_geometry.yaml` | C0 坐标系定义（原点=中圈圆心，+X 指向球门，+Y 为面向 +X 的左侧） | 同上 |
| `camera.yaml` | 相机内参 + 畸变 + 安装外参（已标定） | 全部模式 |
| `c0_field.yaml` / `c0_field_geometry.yaml` | 与 `field.yaml` / `field_geometry.yaml` **内容相同**，为兼容脚本中的显式路径而保留 | `scripts/*` 的默认参数 |
| `field_demo.yaml` / `field_geometry_demo.yaml` | **构造的演示地图**（12 个特征，含 X 型与点球点） | 仅用于 `run.py demo` 的合成自检，**不是 C0 真实场地** |

## ⚠️ 关于 field.yaml 的来源

细则要求 `field.yaml` 使用组织方统一版本。**组织方至今未发布该文件** ——
实测考核仓库 `算法组/Week3-4/` 只有一个 README.md，全库 635 个文件中无任何 yaml。

因此本目录的 `field.yaml` 是**本人用卷尺实测自建**的 C0 地图：
- 8 个场地标志（4 个 L 型 + 4 个 T 型）+ 点球点（按定义固定在原点）
- 坐标定义与细则一致：原点为中圈圆心，+X 指向球门，+Y 为面向 +X 时的左侧，单位米
- 闭环验证误差 5~6 cm

**拿到组长发布的官方版本后**，直接替换 `field.yaml` 与 `field_geometry.yaml` 即可，
代码无需改动（特征坐标从 YAML 读取）。
