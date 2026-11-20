#!/usr/bin/env python3
"""串口链路一键诊断：波特率 / 命令格式 / 通道号 / 回显 全扫一遍。

用法（二选一）：
    python3 serial_probe.py            # 只读：扫波特率和命令格式，不动舵机
    python3 serial_probe.py --move     # 会真的发动作命令，眼睛盯住舵机

判读规则见每一段打印出来的 [结论] 行。
"""
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺 pyserial：pip3 install pyserial  （或 sudo apt install -y python3-serial）")

BAUDS = [115200, 9600, 19200, 38400, 57600, 128000, 250000, 500000, 1000000]

# 动作命令格式候选：(说明, ch/pwm -> 命令)
FORMATS = [
    ("A 幻尔24路PWM  #<ch>P1500T1000!", lambda ch, pwm: "#%03dP%dT1000!" % (ch, pwm)),
    ("B 短格式        #<ch>P1500!", lambda ch, pwm: "#%03dP%d!" % (ch, pwm)),
    ("C 总线舵机(1起)  #<id>P1500T600!", lambda ch, pwm: "#%03dP%dT600!" % (ch + 1, pwm)),
]

QUERIES = [
    ("读ID  #000PID!",   "#%03dPID!"),
    ("读位置 #000PRAD!", "#%03dPRAD!"),
    ("读版本 #000PVER!", "#%03dPVER!"),
]


def list_ports():
    p = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return p


def talk(ser, cmd, wait=0.20):
    """发一条，收 200ms 内的所有回包。"""
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    t0 = time.time()
    buf = b""
    while time.time() - t0 < wait:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.01)
    return buf


def open_port(port, baud):
    return serial.Serial(port, baud, timeout=0.1, write_timeout=1.0)


def main():
    do_move = "--move" in sys.argv
    port = None
    for a in sys.argv[1:]:
        if not a.startswith("--"):
            port = a

    cands = [port] if port else list_ports()
    if not cands:
        sys.exit("没有找到 /dev/ttyUSB* 或 /dev/ttyACM*，先确认 USB-TTL 插好、brltty 已卸载。")

    print("=" * 62)
    print("端口：", ", ".join(cands))
    print("=" * 62)

    hit_baud = None
    hit_cmd = None
    ser = None

    # ---------- 第一轮：扫波特率 + 命令格式，只看有没有回话 ----------
    for baud in BAUDS:
        if hit_baud:
            break
        for pt in cands:
            if hit_baud:
                break
            try:
                s = open_port(pt, baud)
            except Exception as e:
                print("  %s @ %d 打不开：%s" % (pt, baud, e))
                continue
            for label, fmt in QUERIES:
                for ch in (0, 1):
                    cmd = fmt % ch
                    r = talk(s, cmd)
                    if r:
                        print("  [有回话] baud=%d  %s  %s -> %s  (%r)" % (baud, pt, cmd, r.hex(), r))
                        hit_baud, ser = baud, s
                        break
                    print("  [无回话] baud=%d  %s  %s" % (baud, pt, cmd))
                if hit_baud:
                    break
            if not hit_baud:
                s.close()

    if hit_baud:
        print("\n[结论] 通信已通：波特率 = %d，设备 = %s" % (hit_baud, ser.port))
        print("       把 servo_params.yaml 里的 baudrate 改成 %d 即可。" % hit_baud)
    else:
        print("\n[结论] 所有波特率、所有查询命令都没有任何回话。")
        print("       → 说明「板子没有收到你发的字节」，或「板子收到了但不会回复」。")
        print("       优先查物理层：TX/RX 是否交叉、GND 是否共地、插的是不是「用户」那排。")

    # ---------- 第二轮：盲发动作，用眼睛判断发送方向通不通 ----------
    if do_move:
        baud = hit_baud or 115200
        port_to_use = ser.port if ser else cands[0]
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        print("\n" + "=" * 62)
        print("盲发动作测试（baud=%d, port=%s）—— 现在盯住舵机" % (baud, port_to_use))
        print("=" * 62)
        s = open_port(port_to_use, baud)
        for label, build in FORMATS:
            print("\n>>> 测试格式：%s" % label)
            for ch in range(4):
                for pwm in (1500, 1800, 1200, 1500):
                    s.write(build(ch, pwm).encode())
                    s.flush()
                    time.sleep(0.7)
                print("    ch=%d 已发 1500→1800→1200→1500" % ch)
            print("    ↑ 哪一路动了，就记下「格式 %s + ch」" % label[0])
        try:
            s.close()
        except Exception:
            pass

    print("\n完成。把上面全部输出复制给我。")


if __name__ == "__main__":
    main()
