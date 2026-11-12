#!/usr/bin/env bash
# Run inside a new Ubuntu 22.04 WSL instance. Based on official Humble deb docs.
set -Eeuo pipefail
trap 'printf "Installation stopped at line %s. Keep the error output for diagnosis.\n" "$LINENO" >&2' ERR
. /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
    echo 'This installer requires Ubuntu 22.04; no changes were made.' >&2
    exit 2
fi
if (( EUID == 0 )); then
    as_root=()
else
    as_root=(sudo)
    sudo -v
fi
if (( $# > 1 )); then
    echo 'Usage: bash install_ros2_humble.sh [local-ros2-apt-source.deb]' >&2
    exit 2
fi
local_source=${1:-}
if [[ -n "$local_source" && ! -f "$local_source" ]]; then
    echo "Source package not found: $local_source" >&2
    exit 2
fi
export DEBIAN_FRONTEND=noninteractive
"${as_root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update
"${as_root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y locales software-properties-common curl ca-certificates python3
"${as_root[@]}" locale-gen en_US en_US.UTF-8
"${as_root[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
"${as_root[@]}" add-apt-repository -y universe
if [[ -z "$local_source" ]]; then
    source_dir=$(mktemp -d)
    curl --fail --location --retry 2 --connect-timeout 20 --max-time 120 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        -o "$source_dir/release.json"
    python3 - "$source_dir" <<'PY'
import json
from pathlib import Path
import sys
folder = Path(sys.argv[1])
release = json.loads((folder / 'release.json').read_text())
name = f"ros2-apt-source_{release['tag_name']}.jammy_all.deb"
asset = next(a for a in release['assets'] if a['name'] == name)
url = asset['browser_download_url']
if not url.startswith('https://github.com/ros-infrastructure/ros-apt-source/releases/download/'):
    raise ValueError('Unexpected release download URL')
(folder / 'url.txt').write_text(url)
digest = asset.get('digest', '')
(folder / 'digest.txt').write_text(digest)
PY
    source_url=$(<"$source_dir/url.txt")
    local_source="$source_dir/ros2-apt-source.deb"
    curl --fail --location --retry 2 --connect-timeout 20 --max-time 180 "$source_url" -o "$local_source"
    digest=$(<"$source_dir/digest.txt")
    if [[ "$digest" == sha256:* ]]; then
        printf '%s  %s\n' "${digest#sha256:}" "$local_source" | sha256sum --check -
    fi
fi
# Check package identity and architecture; Ubuntu release was checked above.
[[ "$(dpkg-deb -f "$local_source" Package)" == ros2-apt-source ]]
[[ "$(dpkg-deb -f "$local_source" Architecture)" == all ]]
"${as_root[@]}" dpkg -i "$local_source"
"${as_root[@]}" apt-get update
# Official docs recommend upgrading a fresh Jammy system before installing ROS.
"${as_root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
"${as_root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-desktop ros-dev-tools python3-opencv python3-numpy python3-colcon-common-extensions
# ROS setup scripts may reference unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u
[[ "$ROS_DISTRO" == humble ]]
python3 -c 'import sys,cv2,numpy,rclpy; print(sys.version); print("OpenCV",cv2.__version__,"NumPy",numpy.__version__)'
ros2 pkg prefix rclpy
colcon --help > /dev/null
printf '\nROS2 Humble dependencies verified. Custom message build and pub/sub checks are still required.\n'
