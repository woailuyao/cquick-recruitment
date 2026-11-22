#!/usr/bin/env bash
# 一次跑完任务 1 / 任务 2 需要的全部素材。
# 用法：在本文件所在目录（视觉组/Week2/houkehong/）执行
#     bash run_all.sh
# 生成的 png 直接就是提交要求的文件名。

# 任何一步返回非 0 就停下（否则中间失败会被后面的"全部完成"掩盖）
set -euo pipefail

NAME="houkehong"
IMG_DIR="src/green_detector/test_images"
PKG="src/green_detector/green_detector"
MAIN_IMG="${IMG_DIR}/blue_01.png"
LIGHT_IMG="${IMG_DIR}/blue_05.png"

echo "姓名: ${NAME}"; date "+%Y-%m-%d %H:%M:%S"
echo "当前目录: $(pwd)"

if [ ! -f "${MAIN_IMG}" ]; then
  echo "[ERROR] 找不到 ${MAIN_IMG}"
  echo "        先把测试图片按 src/green_detector/test_images/README.md 的命名放进去"
  exit 1
fi

echo
echo "===== 0/4 环境自检 ====="
python3 "${PKG}/check_env.py"

echo
echo "===== 1/4 单张图检测（任务 1 + 任务 2 可视化）====="
python3 "${PKG}/detect_image.py" --image "${MAIN_IMG}" \
  --output "02_结果可视化.png" --mask-output "out/mask_01.png" --name "${NAME}"

echo
echo "===== 2/4 多图批量测试（任务 1，加分项 5 图）====="
python3 "${PKG}/batch_test.py" --image-dir "${IMG_DIR}" \
  --output "02_批量结果.png" --out-dir out --name "${NAME}"

echo
echo "===== 3/4 参数对比：HSV / 面积 / 形态学 ====="
for exp in hsv area morph; do
  python3 "${PKG}/compare_params.py" --experiment "${exp}" --image "${MAIN_IMG}" \
    --output "02_参数对比_${exp}.png" --name "${NAME}"
  echo
done

echo "===== 4/4 光照对比（需要 blue_05.png）====="
if [ -f "${LIGHT_IMG}" ]; then
  python3 "${PKG}/compare_params.py" --experiment light --image "${MAIN_IMG}" \
    --image-b "${LIGHT_IMG}" --output "02_参数对比_light.png" --name "${NAME}"
else
  echo "[SKIP] 没有 ${LIGHT_IMG}，跳过光照对比（这组是可选的）"
fi

echo
echo "全部完成。接下来做 ROS2 部分："
echo "  colcon build --symlink-install --packages-select target_interfaces green_detector"
echo "  source install/setup.bash"
echo "  ros2 interface show target_interfaces/msg/TargetPosition"
echo "  ros2 run green_detector publish_target     # 终端 A"
echo "  ros2 run green_detector subscribe_target   # 终端 B"
