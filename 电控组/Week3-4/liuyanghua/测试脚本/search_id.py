import serial
import time

def scan_zhongling_servos(port, baudrate=115200, timeout=0.05):
    """
    扫描众灵总线舵机 ID
    :param port: 串口，例如 "COM3" 或 "/dev/ttyUSB0"
    :param baudrate: 波特率，众灵舵机常见为 115200
    :param timeout: 每次读取的超时时间（秒），越小扫描越快
    """
    found_ids = []
    
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
    except serial.SerialException as e:
        print(f"❌ 无法打开串口 {port}: {e}")
        return found_ids

    print(f"🔍 正在扫描 {port} @ {baudrate}bps (ID 1~254)...")
    start_time = time.time()

    for servo_id in range(0, 25):
        # 构造 3 位 ID 的查询指令，例如 ID=5 -> "#005PID!"
        cmd = f"#{servo_id:03d}PID!".encode('ascii')
        
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        
        # 短暂等待舵机响应
        time.sleep(timeout)
        response = ser.read(32)  # 读取最多 32 字节
        
        # 检查响应中是否包含对应的 "#XXXP!" 格式
        if response:
            resp_str = response.decode('ascii', errors='ignore')
            # 简单校验：响应中应包含 "#" + 3位ID + "P!"
            expected_marker = f"#{servo_id:03d}P!"
            if expected_marker in resp_str:
                print(f" -> ✅ 发现舵机 ID: {servo_id} | 响应: {resp_str.strip()}")
                found_ids.append(servo_id)

    ser.close()
    elapsed = time.time() - start_time
    print(f"\n扫描完成，耗时 {elapsed:.1f}s。共发现 {len(found_ids)} 个舵机。")
    return found_ids

if __name__ == "__main__":
    # ⚠️ 请按实际情况修改以下两个参数
    SERIAL_PORT = "/dev/ttyUSB0"        # Windows 示例，Linux/Mac 可能是 "/dev/ttyUSB0"
    BAUDRATE = 115200           # 众灵舵机常见波特率，请确认你的型号
    
    online_servos = scan_zhongling_servos(SERIAL_PORT, BAUDRATE)
    if online_servos:
        print(f"在线 ID 列表: {online_servos}")
    else:
        print("未检测到舵机，请检查接线、供电和波特率。")