#!/usr/bin/env bash
# 加分项现场验证（第 3、4 周）
#   全部跑： bash 加分项/verify_bonus.sh
#   单拍一张： bash 加分项/verify_bonus.sh 3     （1=udev 2=pytest 3=PRAD重发 4=看门狗 5=抢占 6=重连）
# 前置：另一个终端已经 ros2 launch cquick_servo servo.launch.py
# 每项开头都会打印「姓名 + 当前时间」，所以每一项单独截一张 04_*.png 都合规。
set -u

TOPIC=/move_servo
TYPE=cquick_servo_interfaces/action/MoveServo
NODE=/servo_driver
TRIG=std_srvs/srv/Trigger
ITEM="${1:-all}"

# 环境兜底：忘了 source 也能跑
# setup.bash 里有未定义变量（COLCON_TRACE），开着 set -u 会被打死，所以临时关掉
set +u
export COLCON_TRACE="${COLCON_TRACE:-}"
command -v ros2 >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
[ -f "$HOME/servo_ws/install/setup.bash" ] && source "$HOME/servo_ws/install/setup.bash"
set -u 2>/dev/null || true

banner() {
    echo ""
    echo "====================================================="
    echo " 姓名: wangxuanhe   时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo " 加分项 ${1} | ${2}"
    echo "====================================================="
}

# 截图闸口：跑到这里脚本会停住，屏幕上正是「该拍的画面」。
# 有 gnome-screenshot 就 2 秒后自动截一张到 ~/桌面，没有就提示按快捷键。
shot() {
    echo ""
    echo "#####################################################"
    echo "#                                                   #"
    echo "#   >>> 现 在 截 图 <<<   存成: $1"
    echo "#   姓名: wangxuanhe   时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "#                                                   #"
    echo "#####################################################"
    if command -v gnome-screenshot >/dev/null 2>&1; then
        echo "（2 秒后自动截一张到 ~/桌面/$1，不用按快捷键）"
        sleep 2
        gnome-screenshot -f "$HOME/桌面/$1" && echo "已保存: $HOME/桌面/$1"
    else
        echo "（没装 gnome-screenshot：按 Ctrl+Shift+PrtSc，或用手机拍）"
    fi
    read -r -p "截好、确认字能看清后按回车继续 > " _
}

# ---------------- 1 udev ----------------
item1() {
    banner "1/6 (+2)" "udev 固定设备名 /dev/cq_servo"
    echo "① 规则内容："
    cat "$(dirname "$0")/99-cquick-servo.rules"
    echo ""
    echo "② 当前设备（应指向 ttyUSB*，组 dialout）："
    ls -l /dev/cq_servo || echo "!! 没有 /dev/cq_servo：规则没装，或 lsusb 里的芯片 ID 与规则不符"
    echo ""
    read -r -p ">>> 拔掉 USB 再插回，插好后立刻按回车（脚本会自己等 udev 建链）：" _
    # 插回后内核要先枚举 ttyUSB*、udev 再建软链，需要几百毫秒到 1 秒，不能马上 ls
    echo "等待 udev 建链（最多 10 s）..."
    for _i in $(seq 1 20); do
        [ -e /dev/cq_servo ] && break
        sleep 0.5
        printf "."
    done
    echo ""
    ls -l /dev/cq_servo || echo "!! 10 s 内没出现，检查：ls /dev/ttyUSB* / lsusb / dmesg | tail -20"
    echo "（对照：内核原始设备名）"
    ls /dev/ttyUSB* 2>/dev/null || echo "  连 ttyUSB* 都没有：USB 没枚举，线或口的问题"
    echo "③ 因为名字固定，servo_params.yaml 里 port 默认值才敢写 /dev/cq_servo"
    shot "04_udev固定设备名.png"
}

# ---------------- 2 pytest ----------------
item2() {
    banner "2/6 (+2)" "pytest 单测（不接硬件可跑）"
    cd "$HOME/servo_ws" || exit 1
    colcon test --packages-select cquick_servo
    colcon test-result --verbose
    echo "预期：flake8 与协议拼串单测全绿（0 errors / 0 failures）"
}

# ---------------- 3 PRAD 重发 ----------------
item3() {
    banner "3/6 (+3)" "PRAD 闭环校验 + 未到位自动重发"
    echo "造「到不了目标」的两种办法，任选一种："
    echo "  A 用手按住舵臂，让它到不了目标（阻力要顶得住）"
    echo "  B 不想用手：目标只比当前位置高 250，100 ms 最多走 60，必然判未到位"
    echo ""
    read -r -p ">>> 选好后按回车，先回中位 1500（给下一步留出上行的行程）：" _
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1500, duration_ms: 1000}" --feedback
    echo ""
    echo "----- 如果上一条到位了，用 B 方案再来一次 -----"
    echo "B 方案：从 1500 往下走 100（1400 这个位置已验证能到）。"
    echo "  100 ms 只挪 ~15 → 差 85 判未到位 → 重发 1500 ms → 到位。"
    echo "  别用上行目标：这台结构的正方向到 ~1500 就顶住了。"
    read -r -p ">>> 按回车发 1400 / 100 ms（必然第一次追不上）：" _
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1400, duration_ms: 100}" --feedback
    echo "驱动端预期日志："
    echo "  id=1 目标 1400，起点 1500，计划 xxxms"
    echo "  id=1 未到位 target=1400 actual=148x，重发一次"
    echo "  id=1 重发后到位 actual=1400"
    shot "04_PRAD重发.png"
}

# ---------------- 4 看门狗 ----------------
item4() {
    banner "4/6 (+3)" "安全看门狗：卸力 + 锁死 + 只能外部确认恢复"
    echo "① 手动注入故障（等价于心跳/执行停摆超过 1 s）："
    ros2 service call "$NODE/watchdog_trip" "$TRIG"
    echo ""
    echo "② 锁死后新 goal 必须被拒（同时驱动端打 Reject goal: watchdog locked）："
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1600, duration_ms: 800}"
    echo ""
    echo "③ 只有 watchdog_reset 能解锁，节点不会自己恢复："
    ros2 service call "$NODE/watchdog_reset" "$TRIG"
    echo ""
    echo "④ 解锁后同一条 goal 应当通过："
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1600, duration_ms: 800}"
    shot "04_看门狗卸力.png"
}

# ---------------- 5 抢占 ----------------
item5() {
    banner "5/6 (+4)" "动作组抢占：不跳变、不先回中位"
    echo "① 先起一个 3 s 的长动作（后台跑）："
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1950, duration_ms: 3000}" &
    LONG_PID=$!
    sleep 1
    echo ""
    echo "② 执行到一半插一个新 goal，角度相反："
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1300, duration_ms: 900}"
    wait "$LONG_PID"
    echo ""
    echo "预期：旧 goal result message=preempted（不 PULK、不回中位）；"
    echo "     新 goal 日志「起点 xxx，计划 xxxms」，按 speed_pwm_per_s 从当前位置接上"
    shot "04_动作组抢占.png"
}

# ---------------- 6 串口重连 ----------------
item6() {
    banner "6/6 (+3)" "串口掉线自动重连（节点不崩）"
    echo ">>> 现在拔掉 USB 转 TTL，等 3 秒"
    read -r -p "拔掉后按回车，让节点去碰断掉的串口：" _
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1600, duration_ms: 3000}" --feedback
    echo ""
    echo "预期：串口掉线，节点继续运行，等待自动重连（没有崩溃、没有退出）"
    read -r -p "插回 USB 后按回车：" _
    sleep 2
    ros2 action send_goal "$TOPIC" "$TYPE" "{id: 1, pwm: 1500, duration_ms: 800}"
    echo "预期：日志出现 串口恢复可用：/dev/cq_servo @115200，这条 goal 正常 success"
    shot "04_串口重连.png"
}

case "$ITEM" in
    1) item1 ;;
    2) item2 ;;
    3) item3 ;;
    4) item4 ;;
    5) item5 ;;
    6) item6 ;;
    all)
        item1; item2; item3; item4; item5; item6
        echo ""
        echo "===================== 验证结束 ====================="
        date "+%Y-%m-%d %H:%M:%S"
        ;;
    *)
        echo "用法: bash 加分项/verify_bonus.sh [1|2|3|4|5|6|all]"
        exit 2
        ;;
esac
