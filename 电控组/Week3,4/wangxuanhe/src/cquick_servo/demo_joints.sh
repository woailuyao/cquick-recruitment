#!/usr/bin/env bash
# 任务三录屏用：10 颗舵机逐个正反转，屏幕上每颗打一行标记，方便镜头对上关节。
# 用法：bash demo_joints.sh            （幅度 ±300，时长 1800 ms，肉眼一眼看得出）
#      STEP=200 DUR=2000 bash demo_joints.sh   （幅度小一点、更稳）
# 前提：servo_driver 已在运行；本脚本只发 action goal。
set -u

STEP=${STEP:-300}          # 摆动幅度（PWM）。±300 ≈ ±27°，镜头里非常明显
DUR=${DUR:-1800}           # 单次时长；PWM 与角度约 1:0.09，300 PWM 走完靠舵机自身速度够用
ACTION=/move_servo
TYPE=cquick_servo_interfaces/action/MoveServo

declare -A NAME=(
  [1]="左·胯外摆" [2]="左·髋俯仰" [3]="左·膝" [4]="左·踝俯仰" [5]="左·踝外摆"
  [6]="右·胯外摆" [7]="右·髋俯仰" [8]="右·膝" [9]="右·踝俯仰" [10]="右·踝外摆"
)

ask() {   # ask <id> <pwm> <标签>
  ros2 action send_goal "$ACTION" "$TYPE" \
    "{id: $1, pwm: $2, duration_ms: $DUR}" 2>&1 \
    | grep -E "Result|success|final_pwm|message" | head -5
}

for id in 1 2 3 4 5 6 7 8 9 10; do
  echo
  echo ">>> 现在动 id=$id  ${NAME[$id]}  +${STEP} <<<"
  ask "$id" $((1500 + STEP)) plus
  sleep 1
  echo ">>> 现在动 id=$id  ${NAME[$id]}  -${STEP} <<<"
  ask "$id" $((1500 - STEP)) minus
  sleep 1
  echo ">>> id=$id 回中立位 1500 <<<"
  ask "$id" 1500 back
  sleep 1
done

echo
echo "===== 10 颗全部演示完毕，全部回 1500 ====="
