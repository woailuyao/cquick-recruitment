#!/usr/bin/env bash
# 任务四录屏专用：一条命令把「卸力 → 拔电池 → 关机」跑成带时间戳的流程，中间停两次等你操作。
#
# 用法：
#   bash demo_poweroff.sh          # 开始录屏后再跑这一条
#   DRY=1 bash demo_poweroff.sh    # 预演：只打到第③步前就退出，不真的关机
#
# 屏幕每行都带 [时:分:秒]，加上开头那行姓名，满足「同屏带姓名+时间戳」。
set -u

ts() { date '+%H:%M:%S'; }
say() { echo "[$(ts)] $*"; }

NAME=${NAME:-王宣赫}
DRY=${DRY:-0}

line() {   # 把命令输出逐行加时间戳，屏幕上时间连续
  while IFS= read -r l; do echo "[$(ts)] $l"; done
}

echo
echo "================================================================"
say "姓名:${NAME}   任务4 安全下电流程   日期:$(date '+%F')"
echo "================================================================"
echo

say "① 卸力：向全部舵机发 PULK（刹车松开，力矩归零）"
ros2 service call /servo_driver/torque_off std_srvs/srv/Trigger "{}" 2>&1 | line
echo
say "① 已完成。现在用转一下膝关节/小腿——能轻松转动 = 力矩确实卸掉了"
read -r -p "[$(ts)] 转完按回车，继续 ② 断动力电 > " _
echo

say "② 断动力电：拔掉电池插头（注意：舵机灯灭，板卡仍由 USB/5V 供电）"
read -r -p "[$(ts)] 插头拔掉后按回车，继续 ③ 关板卡 > " _
echo

if [ "$DRY" = "1" ]; then
  say "DRY=1：到此为止，不执行关机。真实录制时执行下面这条："
  echo
  echo "    sudo shutdown now"
  echo
  exit 0
fi

say "③ 关板卡：sudo shutdown now（直接拔板卡电源会损坏文件系统）"
sudo shutdown now
