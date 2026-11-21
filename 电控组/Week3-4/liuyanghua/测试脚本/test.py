import serial
import time

def send(cmd):
    print(f"发送指令(Hex): {cmd.encode('ascii').hex()}, 发送字符串: {cmd}")
    n = ser.write(cmd.encode('ascii'))
    ser.flush() # 强行刷新发送缓冲区，确保数据真的发出去！
    print(f"实际写入字节数: {n}")

    # 2. 读取返回
    time.sleep(0.1) # 给舵机处理时间
    waiting = ser.in_waiting
    print(f"读取：缓冲区现有字节数: {waiting}")

    if waiting > 0:
        data = ser.read(waiting)
        #print(f"收到原始字节(Hex): {data.hex()}")
        print(f"解析为字符串: {data.decode('ascii', errors='ignore')}")
    else:
        print("没有收到任何数据！")
    print("====================================")

def all_PULK():
    for id in range(0,10):
        cmd = f"#{id:03d}PULK!"
        send(cmd)
        time.sleep(0.05)


# 请确认端口和波特率无误
PORT = "/dev/ttyUSB0" #"/tmp/ttyFAKE" '/dev/ttyUSB1'
BAUD = 115200
ID = 6

ser = serial.Serial(PORT, BAUD, timeout=0.2)

print("清空缓冲区...")
ser.reset_input_buffer()
time.sleep(0.1)

# 1. 发送指令

""""""

#cmd = f"#{ID:03d}P1500T2000!"
#cmd = f"#{ID:03d}PULK!"
#send(f"#{ID:03d}P1500T1500!")
#time.sleep(2)
#send(f"#{ID:03d}P2000T0500!")
all_PULK()
time.sleep(0.6)
#send(f"#{ID:03d}PRAD!")
time.sleep(1.1)
#send(f"#{ID:03d}P1000T0500!")
time.sleep(0.6)
#send(f"#{ID:03d}P1500T0500!")
time.sleep(0.6)
#send("#003P1000T1000!")
"""cmd = f"#{ID:03d}P2100T3000!"
send(cmd)
time.sleep(1)
cmd = f"#{ID:03d}PDST!"
send(cmd)
#time.sleep(1)
cmd = f"#{ID:03d}PRAD!"
send(cmd)
#time.sleep(0.5)
cmd = f"#{ID:03d}P1500T0800!"
send(cmd)
time.sleep(0.9)
cmd = f"#{ID:03d}PRAD!"
send(cmd)"""





ser.close()