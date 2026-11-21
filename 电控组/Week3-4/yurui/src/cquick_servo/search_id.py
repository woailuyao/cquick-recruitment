import re
import sys
import time

try:
    import serial
except ImportError:
    print("缺少串口库，请先运行：sudo apt install python3-serial -y")
    sys.exit(1)


# =====================================================================
# 【区域 1】工具函数
# ---------------------------------------------------------------------
# 两个函数：收回复、认编号。写一次就行，后面直接调用。
# =====================================================================
def read_window(ser, seconds):
    """在 seconds 秒内尽量把舵机的回复都收回来"""
    data = b""
    end = time.time() + seconds
    while time.time() < end:
        n = ser.in_waiting
        if n:
            data += ser.read(n)
        else:
            time.sleep(0.005)
    return data


def parse_reply(data):
    """从回复里抠出舵机编号。返回 (原始文本, 编号列表)

    舵机回显以 # 开头、紧跟三位数字，比如 #004PID0004!
    但总线回显经常不完整（比如只回来 #004PI），
    所以直接从 # 后面读三位数字最可靠 —— 开头的 004 就是 ID。
    """
    text = data.decode("ascii", "replace").strip()
    ids = []
    for m in re.finditer(rb"#(\d{3})", data):
        ids.append(int(m.group(1)))
    return text, ids


# =====================================================================
# 【区域 2】配置区 —— 所有可能改的东西都集中在这里
# ---------------------------------------------------------------------
# 换串口、改扫描范围，全在这一块改，别动别的地方。
# =====================================================================

# --- 串口 ---
PORT = "/dev/ttyUSB0"     # 虚拟串口写 "/dev/ttyFAKE"，真舵机写 "/dev/ttyUSB0"
BAUD = 115200             # 波特率固定 115200，别改

# --- 扫描范围 ---
START = 0                 # 从几号开始扫（你的舵机从 000 起步，就写 0）
END = 9                  # 扫到几号（想扫全部就写 253）

# --- 每个号等多久（秒）---
WINDOW = 0.10             # 舵机回显慢就把这个调大，比如 0.2

# =====================================================================
# 【区域 3】主程序 —— 循环扫描，打印结果
# ---------------------------------------------------------------------
# 看懂就行，一般不用改。
# =====================================================================
def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"打不开串口 {PORT}：{e}")
        print("先跑 ls /dev/ttyUSB* 看设备名对不对；Permission denied 就重启让 dialout 生效")
        return
    print("=" * 56)
    print(f"开始扫描 {PORT}，波特率 {BAUD}，扫 {START}~{END} 号")
    print("=" * 56)

    found = {}  # 编号 -> 收到几条回复
    for sid in range(START, END + 1):
        ser.reset_input_buffer()
        cmd = f"#{sid:03d}PID!".encode("ascii")
        ser.write(cmd)
        ser.flush()
        data = read_window(ser, WINDOW)
        text, ids = parse_reply(data)

        if not ids:
            continue  # 没回答的号不刷屏

        for rid in ids:
            found[rid] = found.get(rid, 0) + 1
        mark = "  ⚠️ 这一条里有多个回答" if len(ids) > 1 else ""
        print(f"  问 {sid:03d} → 回 [{text}]  ({len(ids)} 条){mark}")

    ser.close()

    print("\n" + "=" * 56)
    print("扫描结果")
    print("=" * 56)
    if not found:
        print("  没有任何舵机回答。依次检查：")
        print("   1) USB 转 TTL 插好没，ls /dev/ttyUSB* 有没有设备")
        print("   2) 波特率是不是 115200")
        print("   3) 有没有共地、有没有供电")
        print("   4) Permission denied → 加 dialout 组后重启")
        return

    for rid in sorted(found):
        n = found[rid]
        if n > 1:
            print(f"  ID {rid:03d}  ⚠️ 重复！有 {n} 个舵机都占着这个号 —— 必须改掉")
        else:
            print(f"  ID {rid:03d}  ✅ 正常（1 个）")

    dups = [i for i, n in found.items() if n > 1]
    print(f"\n一共 {len(found)} 个编号在线；重复 {len(dups)} 个。")
    if dups:
        print("重复的编号：" + ", ".join(f"{i:03d}" for i in sorted(dups)) + " —— 这些要重新设 ID")


if __name__ == "__main__":
    main()
