#!/usr/bin/env bash
# Build the project's two ROS packages as the ordinary Linux user.
set -Eeuo pipefail
if (( EUID == 0 )); then
    echo 'Run as the ordinary Linux user, not root.' >&2
    exit 2
fi
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace=${1:-"$HOME/ros2_ws"}
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'ROS2 Humble installation has not finished.' >&2
    exit 2
fi
mkdir -p "$workspace/src"
for package in target_interfaces color_target_detector; do
    source_dir="$project_dir/src/$package"
    destination="$workspace/src/$package"
    if [[ -e "$destination" || -L "$destination" ]]; then
        if [[ ! -L "$destination" || "$(readlink -f "$destination")" != "$(readlink -f "$source_dir")" ]]; then
            echo "Existing package is not this project's source link: $destination" >&2
            echo 'Inspect it first or select a new workspace; nothing has been overwritten.' >&2
            exit 2
        fi
    else
        ln -s "$source_dir" "$destination"
    fi
done
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"
colcon build --symlink-install --packages-select target_interfaces color_target_detector
set +u
source "$workspace/install/setup.bash"
set -u
ros2 interface show target_interfaces/msg/TargetPosition
ros2 pkg executables color_target_detector
printf '\nBuild complete. New terminals must source /opt/ros/humble/setup.bash and %s/install/setup.bash\n' "$workspace"
