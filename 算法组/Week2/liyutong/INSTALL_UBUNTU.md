# Ubuntu 22.04 / ROS2 Humble 安装步骤

此文档依据 ROS2 Humble 官方 Ubuntu deb 安装说明：
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

WSL 安装参考：https://learn.microsoft.com/en-us/windows/wsl/install

下面命令在 Ubuntu 终端执行，只有第一步在 Windows PowerShell。安装过程中需要的 Linux 密码由你在本地输入，不发给他人。

项目另提供 tools/install_ros2_humble.sh，完成 Ubuntu 首次配置后可从本项目目录执行 `bash tools/install_ros2_humble.sh`。脚本会安装官方软件源、更新新系统并安装 Humble 与 OpenCV 依赖，失败时停止。已在本机 Ubuntu 22.04 中运行成功，验证 Python 3.10.12、OpenCV 4.5.4、NumPy 1.21.5 和 rclpy 可导入；以下手动步骤可用于逐项排查。


## 1. 检查 WSL（Windows PowerShell）

```powershell
wsl --list --verbose
```

应看到 Ubuntu-22.04 且 VERSION 为 2。安装提示重启时先重启。首次启动：

```powershell
wsl -d Ubuntu-22.04
```

按终端提示创建 Linux 用户和密码。输入密码时不会显示字符，属于正常行为。

## 2. 检查 Ubuntu 与 locale

```bash
cat /etc/os-release
uname -m
locale
```

VERSION_ID 必须是 22.04。若不是，请先停止，避免混用软件源。以下步骤面向全新安装的 Ubuntu 22.04。

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl ca-certificates python3
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

## 3. 配置官方 ROS 软件源

逐段执行并检查输出。通过官方 GitHub API 获取 ros-apt-source 最新版本号，再下载其 Ubuntu jammy 包；不是从网页提取并执行脚本。

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
printf '%s\n' "$ROS_APT_SOURCE_VERSION"
```

确认上一步输出非空版本号后：

```bash
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

如果网络报错，保留完整输出排查，不要跳过失败步骤。

## 4. 更新系统并安装 ROS

官方建议先更新系统，尤其新安装的 Ubuntu 22.04 的 systemd/udev 相关包。下面 apt upgrade 会显示拟修改内容，阅读后确认：

```bash
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools python3-opencv python3-numpy python3-colcon-common-extensions
```

## 5. 验证环境

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
python3 -c 'import sys,cv2,numpy,rclpy; print(sys.version); print("OpenCV",cv2.__version__,"NumPy",numpy.__version__)'
ros2 pkg prefix rclpy
colcon --help
```

ROS_DISTRO 应是 humble。先在每个新终端手动 source，确认可用后再考虑加入 ~/.bashrc。避免激活 Windows 项目虚拟环境、Conda 或另一个 ROS 版本来编译 Humble 包。

ROS2 官方示例可用两个终端测试（两个终端都先 source）：

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_py listener
```

完成后按项目 README 构建接口包与视觉节点包。安装完成不等于通信任务已验收，仍需实际测试自定义消息和检测结果。
