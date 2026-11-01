# liyutong 的截图清单

截图已于 2026-11-01 在本机实际运行生成，保存于 outputs/screenshots。终端截图包含 `姓名: liyutong`、真实 `date` 时间戳以及对应命令结果；文件内容来自真实节点输出，未使用合成图代替。

## 已生成

| 文件 | 类型 | 内容 |
|---|---|---|
| 01_detection.png | 终端截图 | 五张真实图片检测输出，含坐标、面积与未检测到提示 |
| 01_five_photo_summary.jpg | 合成图 | 五图轮廓与坐标汇总（带姓名与生成时间标注） |
| 02_result_comparison.png | 合成图 | 清晰图与同色背景图的原图/标注对比 |
| 02_parameter_comparison.jpg | 合成图 | 饱和度下界与最小面积两项单变量对比 |
| 03_interface_show.png | 终端截图 | ros2 interface show 输出四字段 |
| 03_pub_sub.png | 终端截图 | 发布与订阅节点同时运行，count=14，双端状态与坐标一致 |
| 03_topic_info.png | 终端截图 | 话题类型、发布者数量与一条实际消息 |
| 04_parameter_change.png | 终端截图 | 运行中将 min_area 改为 200000 后，检测由 True 变 False |

说明：
- 终端截图是真实窗口捕获，含姓名和实时时间。
- 标为合成图的三张是用真实输出文件排版生成的图，不是屏幕截图；图中同时打印了姓名与生成时间，用于阅读与对比，不作为“终端运行截图”使用。

## 复现命令

Windows PowerShell 进入 Ubuntu：

```powershell
wsl -d Ubuntu-22.04 --cd /mnt/c/Users/liyut/Documents/deepseek-harness/default-workspace/week2_color_target
```

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. 检测输出（01）：

```bash
echo "姓名: liyutong"; date "+%Y-%m-%d %H:%M:%S"
ros2 run color_target_detector detect_images data/images/clear.jpg data/images/small.jpg data/images/background.jpg data/images/lighting.jpg data/images/absent.jpg --output outputs/screenshot_detection --hsv-lower 5 200 80 --min-area 100
```

2. 消息定义（03_interface_show）：

```bash
echo "姓名: liyutong"; date "+%Y-%m-%d %H:%M:%S"
ros2 interface show target_interfaces/msg/TargetPosition
```

3. 发布订阅同屏（03_pub_sub）：

```bash
echo "姓名: liyutong"; date "+%Y-%m-%d %H:%M:%S"
bash tools/shots/pubsub_demo.sh
```

4. 话题检查与实际消息（03_topic_info）：

```bash
echo "姓名: liyutong"; date "+%Y-%m-%d %H:%M:%S"
bash tools/shots/topic_demo.sh
```

5. 动态参数（04_parameter_change）：

```bash
echo "姓名: liyutong"; date "+%Y-%m-%d %H:%M:%S"
bash tools/shots/param_demo.sh
```

6. 两张合成对比图（02_*）：

```bash
.venv/Scripts/python.exe tools/make_view_sheet.py     # Windows
.venv/Scripts/python.exe tools/make_deliverable_figures.py
```

Ubuntu 下可用 `python3` 并安装 python3-opencv 执行同样脚本。

## 提交前核对

个人目录为 视觉组/Week2/liyutong。提交内容包括真实图片、标注结果、上述文档、src 与截图；不要提交 .venv、downloads、build、install、log 或 WSL 虚拟磁盘。
