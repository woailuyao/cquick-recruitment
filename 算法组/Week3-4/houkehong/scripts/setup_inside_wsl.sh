#!/usr/bin/env bash
# 在 Ubuntu 22.04 (WSL2) 内部执行：自动选可用镜像 -> 换源 -> 基础工具 -> ROS2 Humble
# 由 setup_wsl.ps1 以 root 身份调用，不要直接手动跑。
#
# 注意：清华镜像对本机 IP 返回 403（Windows 侧却正常），所以这里不写死单一镜像，
#       而是按候选列表逐个探测，选第一个能返回 200 的。
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

log() { echo "[$(date +%H:%M:%S)] $*"; }

UBUNTU_CANDIDATES=(
  "https://mirrors.ustc.edu.cn/ubuntu"
  "https://mirrors.aliyun.com/ubuntu"
  "https://mirrors.huaweicloud.com/ubuntu"
  "https://mirrors.cloud.tencent.com/ubuntu"
  "http://archive.ubuntu.com/ubuntu"
)
ROS2_CANDIDATES=(
  "https://mirrors.ustc.edu.cn/ros2/ubuntu"
  "http://packages.ros.org/ros2/ubuntu"
)

# ---------------------------------------------------------------- 选镜像
log "=== [1/8] 探测可用镜像 ==="
UBUNTU_MIRROR=""
for u in "${UBUNTU_CANDIDATES[@]}"; do
  code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 15 "${u}/dists/jammy/Release" 2>/dev/null || echo 000)
  if [ "$code" = "200" ]; then UBUNTU_MIRROR="$u"; break; fi
  log "  不可用 ($code): $u"
done
[ -n "$UBUNTU_MIRROR" ] || { log "!! 没有任何 Ubuntu 镜像可用，检查网络"; exit 10; }
log "  Ubuntu 镜像: $UBUNTU_MIRROR"

ROS2_MIRROR=""
for u in "${ROS2_CANDIDATES[@]}"; do
  code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 15 "${u}/dists/jammy/InRelease" 2>/dev/null || echo 000)
  if [ "$code" = "200" ]; then ROS2_MIRROR="$u"; break; fi
  log "  不可用 ($code): $u"
done
[ -n "$ROS2_MIRROR" ] || { log "!! 没有可用 ROS2 镜像"; exit 11; }
log "  ROS2   镜像: $ROS2_MIRROR"

# ---------------------------------------------------------------- 换源
log "=== [2/8] 备份并替换 apt 源 ==="
cp /etc/apt/sources.list "/etc/apt/sources.list.bak.$(date +%s)"
cat > /etc/apt/sources.list <<EOF
deb $UBUNTU_MIRROR jammy main restricted universe multiverse
deb $UBUNTU_MIRROR jammy-updates main restricted universe multiverse
deb $UBUNTU_MIRROR jammy-backports main restricted universe multiverse
deb $UBUNTU_MIRROR jammy-security main restricted universe multiverse
EOF
cat /etc/apt/sources.list

log "=== [3/8] apt update ==="
apt-get update -y

log "=== [4/8] 基础工具 ==="
apt-get install -y --no-install-recommends \
  curl wget gnupg lsb-release ca-certificates software-properties-common \
  build-essential cmake git unzip zip \
  python3-pip python3-venv python3-dev \
  v4l-utils usbutils pkg-config \
  locales

log "=== [5/8] locale ==="
sed -i 's/^# *\(en_US.UTF-8\)/\1/' /etc/locale.gen || true
sed -i 's/^# *\(zh_CN.UTF-8\)/\1/' /etc/locale.gen || true
locale-gen || true
update-locale LANG=en_US.UTF-8 || true

log "=== [6/8] 导入 ROS 签名 key + 配置 ROS2 源 ==="
install -d /usr/share/keyrings
KEYSRC=/mnt/d/vision_ws/_setup/ros.key
if [ -f "$KEYSRC" ]; then
  cp "$KEYSRC" /usr/share/keyrings/ros-archive-keyring.gpg
  log "  使用 Windows 侧预下载的 ros.key"
else
  curl -sSL https://gh-proxy.com/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  log "  从 gh-proxy 下载 ros.key"
fi
ARCH="$(dpkg --print-architecture)"
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ROS2_MIRROR jammy main" \
  > /etc/apt/sources.list.d/ros2.list
cat /etc/apt/sources.list.d/ros2.list
apt-get update -y

log "=== [7/8] 安装 ROS2 Humble Desktop（约 1.5GB，最慢的一步）==="
apt-get install -y ros-humble-desktop ros-dev-tools

log "=== [8/8] 配置 shell 环境 ==="
cat > /etc/profile.d/ros2.sh <<'EOF'
# ROS2 Humble 环境
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
fi
EOF
chmod 644 /etc/profile.d/ros2.sh

log "=== 验证 ==="
set +u
. /opt/ros/humble/setup.bash
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "ROS_VERSION=${ROS_VERSION:-}"
echo "包数量: $(ros2 pkg list 2>/dev/null | wc -l)"
echo "=== 已完成 ==="
