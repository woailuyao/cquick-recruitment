# Task3 ROS2 接口包

把 `scr/task3.py` 的检测与定位链路封装成 ROS2 话题，不复制 `task3.py`、`marker.pt`、`ball.pt`。

## 包结构

```text
scr/
├─ task3.py                 # 原有算法，节点运行时按路径加载
├─ marker.pt / ball.pt      # 原有权重，节点直接引用
├─ task3_interfaces/        # ament_cmake：Localization / MarkerDetection / BallDetection
└─ task3_ros2/              # ament_python：task3_node
```

## 依赖安装

`ultralytics` 不是 ROS2/rosdep 包，`colcon build` 不会自动安装它。必须在
`ros2 run` 使用的同一个 Python 环境里安装：

```bash
python3 -m pip install --user ultralytics
python3 -c "import ultralytics, torch; print(ultralytics.__version__, torch.__version__)"
```

如果 Ubuntu 提示 `externally-managed-environment`，二选一：

```bash
# 方式 A：允许用户级 pip 安装（简单）
python3 -m pip install --user --break-system-packages ultralytics

# 方式 B：建一个带系统包访问权限的 venv，再在里面装
python3 -m venv --system-site-packages ~/task3_venv
source ~/task3_venv/bin/activate
python3 -m pip install ultralytics
```

CPU 机器想少下一点，可以先装 CPU 版 torch，再装 ultralytics：

```bash
python3 -m pip install --user torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --user ultralytics
```

## WSL 里的摄像头

WSL2 默认**看不到 Windows 的摄像头**，`/dev/video0` 不存在时节点报
`未发现可用摄像头` 是正常的，不是代码问题。两条路：

### 方式 A：USB 摄像头直通 WSL（推荐给 USB 摄像头）

Windows 管理员 PowerShell：

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
usbipd attach --wsl --busid <USB设备的BUSID>
```

然后在 WSL 里确认：

```bash
ls /dev/video*
```

出现 `/dev/video0` 后直接运行节点，`source` 保持默认 `auto` 即可。

注意：`usbipd` 只能直通 USB 设备，笔记本内置摄像头不行。

### 方式 B：Windows 端转 MJPEG 流（内置/USB 都行）

Windows 上先装 OpenCV：

```powershell
python -m pip install opencv-python
python tools/windows_camera_bridge.py --index auto --port 8080
```

WSL 里用 Windows 网关地址读流：

```bash
host_ip=$(ip route show default | awk '{print $3}')
cd /mnt/c/Users/cijiz/cquick-recruitment/算法组/Week3-4/sumaolin/scr
ros2 run task3_ros2 task3_node --ros-args \
  -p task3_root:=$PWD \
  -p source:=http://$host_ip:8080/video
```

如果 Windows 防火墙弹窗，允许 Python 访问专用网络。`--index` 可以指定
`0/1/2`，`auto` 会选最后一个可用摄像头（通常 USB 排在内置后面）。

## 编译

在 WSL2 / ROS2 Humble 中，从 `scr/` 目录编译；构建产物放 WSL 本地盘，避免写 `/mnt/c`：

```bash
cd /mnt/c/Users/cijiz/cquick-recruitment/算法组/Week3-4/sumaolin/scr
colcon build \
  --base-paths . \
  --build-base ~/ros2_ws/build \
  --install-base ~/ros2_ws/install \
  --packages-select task3_interfaces task3_ros2
source ~/ros2_ws/install/setup.bash
```

节点没有把 `task3.py` / `.pt` 安装进包里，所以运行时必须让节点知道 `scr` 在哪里。

## 运行

推荐在 `scr/` 下运行（默认 `task3_root` 就是当前目录）：

```bash
cd /mnt/c/Users/cijiz/cquick-recruitment/算法组/Week3-4/sumaolin/scr
source ~/ros2_ws/install/setup.bash
ros2 run task3_ros2 task3_node --ros-args -p task3_root:=$PWD
```

如果不在 `scr/` 下运行，显式指定：

```bash
ros2 run task3_ros2 task3_node --ros-args \
  -p task3_root:=/mnt/c/Users/cijiz/cquick-recruitment/算法组/Week3-4/sumaolin/scr
```

查看消息：

```bash
ros2 interface show task3_interfaces/msg/Localization
ros2 topic info /task3/localization
ros2 topic echo /task3/localization
ros2 run task3_ros2 task3_listener
```

## 常用参数

```bash
# 指定摄像头编号 / auto / usb / builtin
-p source:=1

# 指定模型、场地、标定
-p marker_model:=/path/to/scr/marker.pt
-p ball_model:=/path/to/scr/ball.pt
-p field:=/path/to/config/field.yaml
-p calib:=/path/to/config/camera_intrinsics.yaml

# 位姿算法：pnp（默认，不依赖安装外参）或 plane（旧方案）
-p pose_mode:=pnp

# 是否弹出 task3 分屏窗口（WSL 需要 GUI/X11；默认 false）
-p display:=true
```

## 消息字段

`Localization`：

| 字段 | 含义 |
|---|---|
| `header` | 时间戳、frame_id |
| `valid` / `status` | 本帧位姿是否有效及原因 |
| `pose_mode` | `pnp` / `plane` |
| `marker_count` | 参与求解的内点数 |
| `residual` / `residual_unit` | PnP 为 px，plane 为 m |
| `x_camera_map` / `y_camera_map` / `yaw_camera_map` | 相机在地图中的位姿 |
| `fps` | 节点处理帧率 |
| `markers[]` | L/T/X 检测：像素坐标、置信度；plane 模式额外带地面 x/y |
| `balls[]` | 足球检测：置信度、中心像素、bbox |

## 注意

- 节点使用与 `ros2` 相同的 Python 解释器，需要该环境已安装 `ultralytics`、`opencv-python`、`numpy`、`pyyaml`。
- `task3.py` 里的相机标定、PnP、field.yaml 解析、绘制函数都是直接复用，没有第二份实现。
- 如果 `task3_root` 不对，节点会直接报找不到 `task3.py`，不会静默使用旧文件。
