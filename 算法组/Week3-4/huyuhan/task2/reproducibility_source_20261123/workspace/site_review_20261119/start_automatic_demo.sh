#!/usr/bin/env bash
set -euo pipefail
site="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
root="$(cd -- "$site/.." && pwd -P)"
bridge="${CQUICK_BRIDGE_ROOT:-/mnt/c/Users/13991/CquickCameraBridge}"
py="$root/.venv/bin/python"
case "${1:-}" in
  live)
    shift
    if [[ ! -d "$bridge/shared" ]]; then
      printf '%s\n' 'Windows v2 共享目录不存在。' >&2
      exit 2
    fi
    mkdir -p "$root/lab_launch_20261119/runs"
    exec 9>"$root/lab_launch_20261119/runs/.reader.lock"
    flock -n 9 || { printf '%s\n' '桥接已有读取程序，请先退出。' >&2; exit 2; }
    "$py" -B "$root/lab_launch_20261119/preflight.py" "$bridge" --conflicts
    exec "$py" -B "$site/automatic_demo/run_auto.py" --live "bridge:$bridge/shared" "$@"
    ;;
  *)
    exec "$py" -B "$site/automatic_demo/run_auto.py" "$@"
    ;;
esac
