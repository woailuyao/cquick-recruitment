# 任务 3 运行步骤（在虚拟机里照着做）

> 前提：虚拟机里已装好 ROS2 Humble（`printenv ROS_DISTRO` 输出 `humble`）。
> 如果你装的是 foxy/iron/jazzy，把下面所有 `humble` 换成你的发行版名即可。

---

## 0. 把整个 week2test 文件夹弄进虚拟机

推荐放在家目录，路径不要带中文：

```bash
# 目标位置
~/week2test
```

三种搬法任选：
- 虚拟机共享文件夹（VMware 的 `/mnt/hgfs/...`、VirtualBox 的 `/media/sf_...`）里直接 `cp -r`
- 直接把文件夹从 Windows 拖进 Ubuntu 桌面
- 或者用 git / U 盘

搬完确认这三样都在：

```bash
cd ~/week2test
ls image/                      # 1.png 2.png 3.png
ls ros2_ws/src/cquick_interfaces/
ls task1.py task3.py
```

---

## 1. 装依赖（只装一次）

```bash
sudo apt update
sudo apt install -y python3-opencv python3-colcon-common-extensions

# 确认 ROS2 自带的 python 能 import cv2
python3 -c "import cv2; print(cv2.__version__)"
```

> ⚠️ 如果你平时用 conda/venv，先 `conda deactivate`，保证 `which python3` 是 `/usr/bin/python3`。
> ROS2 节点只认系统那个 python3。

---

## 2. 编译接口包（只编一次，改了 .msg 才需要重编）

```bash
cd ~/week2test/ros2_ws
colcon build --packages-select cquick_interfaces
source install/setup.bash
```

编译成功会看到 `Summary: 1 package finished`。

---

## 3. 检查消息是否生成

```bash
source /opt/ros/humble/setup.bash
source ~/week2test/ros2_ws/install/setup.bash
ros2 interface show cquick_interfaces/msg/TargetPosition
```

应该看到四行字段：

```text
bool detected
float32 u
float32 v
float32 area
```

---

## 4. 开两个终端跑起来

**终端 A（发布端）**

```bash
source /opt/ros/humble/setup.bash
source ~/week2test/ros2_ws/install/setup.bash
cd ~/week2test
python3 task3.py pub
```

**终端 B（订阅端）**

```bash
source /opt/ros/humble/setup.bash
source ~/week2test/ros2_ws/install/setup.bash
cd ~/week2test
python3 task3.py sub
```

终端 A 会先打印四帧的检测结果（1.png / 2.png / 3.png / 负样本），然后每 0.5 秒发一条；
终端 B 每条打印一行，`#004` 那条是 `detected=False`。按 2 Hz，5 秒就有 10 条以上。

按 `Ctrl-C` 退出时两边都会打印统计（发了多少条 / 收了多少条）。

---

## 5. 验证话题（第三个终端）

```bash
source /opt/ros/humble/setup.bash
source ~/week2test/ros2_ws/install/setup.bash

ros2 topic list
ros2 topic info /target_position      # Type 应是 cquick_interfaces/msg/TargetPosition
ros2 topic echo /target_position      # 看原始消息内容
ros2 topic hz /target_position        # 应该在 2.0 左右
```

---

## 6. 截图（交作业用）

每张关键图前先执行，把姓名和时间戳带进画面：

```bash
echo "姓名: tangziyi"; date "+%Y-%m-%d %H:%M:%S"
```

需要四张：
1. `ros2 interface show` 的输出
2. 终端 A + 终端 B 并排（能同时看到发布和订阅）
3. `ros2 topic info /target_position`
4. `ros2 topic echo /target_position` 里一条真实检测结果

---

## 7. 常见报错

| 报错 | 原因 / 解决 |
| --- | --- |
| `ros2: command not found` | 没 source：`source /opt/ros/humble/setup.bash` |
| `No module named 'rclpy'` | 同上，或者用错了 python（conda/venv 里的没有 rclpy） |
| `No module named 'cquick_interfaces'` | 第 2 步没编译，或没 `source ~/week2test/ros2_ws/install/setup.bash` |
| `Package 'cquick_interfaces' not found` | 编译目录不对：必须在 `~/week2test/ros2_ws` 里 build |
| `No module named 'cv2'` | `sudo apt install python3-opencv` |
| `ros2 interface show` 报找不到包 | source 顺序：先 `/opt/ros/humble`，再 `install/setup.bash` |
| B 端一条都收不到 | 两个终端是不是都 source 了 install/setup.bash？`ros2 topic list` 里有没有 `/target_position` |
| 图片读不到 | `echo $?` 看终端 A 开头那几行，`图片目录：` 指到哪；用 `-p image_dir:=/绝对路径` 覆盖 |

覆盖图片目录（可选）：

```bash
python3 task3.py pub --ros-args -p image_dir:=$HOME/week2test/image -p publish_rate:=2.0
```
