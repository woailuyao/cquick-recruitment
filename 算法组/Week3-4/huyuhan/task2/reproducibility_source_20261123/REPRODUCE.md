# Task 2 — 原始源码可追溯归档及复现边界

本目录由 `cquick_pr282_v3_one_shot.py` 从本人 WSL 原始工程**复制实际存在的源码和配置**生成；不重新实现算法、不改变训练结果，也不宣称在异机上已完成全部运行验证。

## 来源与目录

- `workspace/site_review_20261119/automatic_demo/`：自动定位入口、地图匹配调度；
- `workspace/site_review_20261119/integrated_demo/`：检测器、可视化及配套辅助模块；
- `workspace/site_review_20261119/formal_map/`：场地内参及真实测量地图；
- `workspace/task2_manual_pnp_v1/`：相关几何辅助模块；
- `workspace/task2_lab_night_v1_20261116/src/`、`config/`：真实场地几何细化及历史相机外参；
- `workspace/site_review_20261119/start_automatic_demo.sh`：原工程启动脚本（如存在）；
- `windows_bridge/`：用户本机 Windows 相机协议代码（若找到）；
- `LOCAL_SOURCE_AUDIT.json`：文件 SHA256、源文件缺口、正式权重 SHA256 比对状态。

## 运行环境

原始系统 WSL Ubuntu 22.04、Python 3.10、OpenCV、NumPy、SciPy、PyYAML、Ultralytics、PyTorch（CUDA 依赖按设备安装），ROS2 Humble 为可选离线接口实验。不要直接在 ROS2 系统 Python 中盲装不匹配的 Torch。

## 最小检查：编译语法（不等于训练或在线成功）

```bash
python3 -m compileall -q workspace/site_review_20261119/automatic_demo workspace/site_review_20261119/integrated_demo workspace/task2_manual_pnp_v1 workspace/task2_lab_night_v1_20261116/src
```

## 在原始 WSL 设备核验实际运行

保留原来的 `.venv`、标定及权重路径后，在个人原工程目录中运行：

```bash
CQ=/home/hu_yuhan/Cquick/Week3-4
source "$CQ/.venv/bin/activate"
python "$CQ/site_review_20261119/automatic_demo/run_auto.py" --check --approximate-mount
```

`--approximate-mount` 只用于重现 2026-11-22 历史近似安装测试，**不能声明已通过当前安装米制误差验收**。无权重/视频/桥接协议时会报错，不能用假权重或合成输入冒充真实演示。

原始工程必须在固定相机、当前安装外参合法的条件下另行执行 C0 三点误差复核。

## 数据与模型

- 原始图片/标签不随仓库上传，数据摘要见 `../../dataset/dataset_summary.md`；
- 正式模型不纳入 Git，需提供组内指定共享存储可访问链接；
- 正式权重历史冻结 SHA256：`2603112118bcaf24b050f3870292dd8f23ba491bd407f7eae83f516ff6c82418`；
- 本地权重检查情况以 `LOCAL_SOURCE_AUDIT.json` 为准，**不能把本地路径说成公共下载链接**。

## 已知缺陷及不应夸大的结论

1. 当前物理安装外参尚未独立核对。P1/P2/P3 的 2026-11-19 历史 `task2_manual_pnp_v1` 结果已按用户于 2026-11-23 确认的 +X/+X 半场条件完成位置误差及近似朝向差计算；这不等于新版 YOLO 自动定位在当前安装下通过 C0 独立误差验收。伞杆方向仅为近似 yaw 参考，角度不确定度未可靠量化。
2. 只有 96 张正式场地标志数据，低于 300 张建议量；307 是标注目标数。
3. ROS2 离线回放通过不等于实时相机闭环；IRLS 合成点对照未部署到自动定位主链路。
4. 这里的 Python 语法编译及文件哈希检查也**不等于完整、可复现的端到端实机验收**。


## 2026-11-23 历史 PnP 三点核定补充

用户已逐点确认 2026-11-19 历史实验的 P1/P2/P3 相机垂足与伞尖 A 均位于 +X 半场。采用 +X/+X 条件的结果：

| 点位 | 固定窗有效帧 | 位置误差中位数 | 位置误差 P95 | 近似朝向差中位数 |
|---|---:|---:|---:|---:|
| P1 | 178/178 | 0.3772 m | 0.3781 m | 4.82° |
| P2 | 179/179 | 0.1722 m | 0.1724 m | 3.19° |
| P3 | 179/179 | 0.1796 m | 0.1798 m | 6.74° |

完整派生统计见 `../evidence/C0_P1_P2_P3/history_20261119/C0_three_point_confirmed_summary_20261123.md` 及对应 CSV、逐点核定报告。

**解释边界：**
- 以上均为 2026-11-19 历史人工初始化＋光流＋PnP 固定静止窗口结果，不是新版 YOLO 自动定位结果。
- 位置误差按逐帧程序位置与现场参考坐标的欧氏距离计算。
- 近似朝向差以相机垂足指向伞尖 A 的伞杆方向为参考；角度不确定度尚未可靠量化。
- XY jitter 是静止窗口内的短时稳定性，不是绝对定位误差。
- 2026-11-23 新截图属于另一次运行，尤其 P1 的 X 符号差异仍需单独核查，不能与 11 月 19 日数据混算。

## 正式权重获取状态

正式模型权重历史 SHA256：
`2603112118bcaf24b050f3870292dd8f23ba491bd407f7eae83f516ff6c82418`（6,484,586 bytes）。

原始 WSL 工作区已核验该文件存在且哈希匹配，但权重未纳入 Git，`LOCAL_SOURCE_AUDIT.json` 中公共下载地址为 null。需要在指定共享存储提供真实可访问的权重文件或链接，其他设备才能完成完整模型推理复现。不得把本机绝对路径当作公共下载地址。
