#!/usr/bin/env python3
# ↑ shebang：告诉 Linux 用 python3 运行这个文件（配合 chmod +x 才能直接跑）

import time
import threading                                 # 线程锁：串口是单行道，多线程必须排队
import serial                                    # 串口库（pyserial）
import rclpy                                     # ROS2 的 Python 客户端库
from rclpy.node import Node                      # 节点基类
from rclpy.action import ActionServer, CancelResponse, GoalResponse   # Action 服务器 + 两个应答枚举
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup   # 可重入组(让cancel能进来) + 心跳专属组
from rclpy.executors import MultiThreadedExecutor            # 多线程执行器（关键！）
from rcl_interfaces.msg import SetParametersResult           # 参数校验回调的返回值类型
from std_msgs.msg import Header #心跳发布
from cquick_servo_interfaces.msg import ServoCommand         # 我们自己定义的 msg
from cquick_servo_interfaces.action import MoveServo         # 我们自己定义的 action
START = '#'                 # 指令起始符
END = '!'                   # 指令结束符
P = 'P'                     # 位置指令标识
T = 'T'                     # 时间指令标识
READ_POS = 'PRAD'           # 读位置
RESTORE_TORQUE = 'PULR'     # 恢复扭力（动之前必须发）
NEUTRAL_PWM = 1500          # 中立位
class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')     # 节点名 = 'servo_driver'（和 yaml 顶层 key 一致）
        self.declare_parameter('port', '/dev/cq_servo')      # 串口设备名
        self.declare_parameter('baudrate', 115200)          # 波特率
        self.declare_parameter('pwm_min', 900)              # 安全下限
        self.declare_parameter('pwm_max', 2100)             # 安全上限
        self.declare_parameter('default_duration_ms', 800)  # 客户端没给 duration 时用
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])  # 本机有哪些舵机
        self.declare_parameter('startup_reset', True)       # 启动时是否回中位
        
        self.add_on_set_parameters_callback(self._on_param_change)
        
        self.ser = None
        self._ser_lock = threading.Lock()    # 串口互斥锁：一次只允许一个线程做完整的收发事务
        self._open_serial()
        self.state_pub = self.create_publisher(Header, '/servo/state', 10)
        self._hb_group = MutuallyExclusiveCallbackGroup()   # 心跳专属组：谁也挤不掉它
        self.create_timer(0.5, self._publish_heartbeat, callback_group=self._hb_group)   # 每 0.5 秒发一次心跳
        self.create_subscription(ServoCommand, '/servo/command', self._command_cb, 10)
         # 参数依次是：消息类型、话题名、收到后调哪个函数、队列长度
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10) 
        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',          # 本节点、action 类型、action 名
            execute_callback=self._execute_cb,      # 真正干活的主回调
            goal_callback=self._goal_cb,            # 收 goal 时先校验，能拒绝
            cancel_callback=self._cancel_cb,        # 取消请求回调
            callback_group=ReentrantCallbackGroup(),  # 可重入，让 cancel 能进来
        )
        if self.get_parameter('startup_reset').value:   # 实时读参数，不存副本
            self._startup_reset()
        
    def _on_param_change(self, params):
        for p in params:                       # 可能一次改多个参数
            if p.name == 'pwm_max':
                v = int(p.value)
                pwm_min = self.get_parameter('pwm_min').value   # 实时读另一个参数
                if not (500 <= v <= 2500):     # 电气极限范围校验
                    return SetParametersResult(successful=False, reason=f'pwm_max 超出 500~2500：{v}')
                if v <= pwm_min:               # 互相打架的组合校验
                    return SetParametersResult(successful=False, reason=f'pwm_max({v}) 必须大于 pwm_min({pwm_min})')
            elif p.name == 'pwm_min':
                v = int(p.value)
                pwm_max = self.get_parameter('pwm_max').value
                if not (500 <= v <= 2500):
                    return SetParametersResult(successful=False, reason=f'pwm_min 超出 500~2500：{v}')
                if v >= pwm_max:
                    return SetParametersResult(successful=False, reason=f'pwm_min({v}) 必须小于 pwm_max({pwm_max})')
            elif p.name == 'baudrate':
                if int(p.value) not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(successful=False, reason=f'不认识的波特率：{p.value}')
        return SetParametersResult(successful=True)   # 返回 True 才真正写进去
    def _open_serial(self):
        port = self.get_parameter('port').value      # 实时读，不是写死的 /dev/ttyUSB0
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f'串口已打开：{port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'打开串口失败 {port}：{e}')
    def _force_close(self):
        """任何串口异常后统一走这里：关门、置空，下次调用自动重连。
        关键：必须把 self.ser 置 None，否则下一个 goal 会在半死的串口对象上继续炸。"""
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
    def _write_cmd(self, sid, cmd):
        """拼 ASCII 指令并发送。ID 固定 3 位补零。掉线自动重连。"""
        s = f'{START}{sid:03d}{cmd}{END}'     # 例：# + 001 + P1500T0800 + !
        if self.ser is None:
            self._open_serial()               # 掉线后先尝试重连
        if self.ser is None:
            self.get_logger().warn('串口未连接，跳过发送')
            return None
        try:
            self.ser.write(s.encode('ascii')) # 字符串→字节→写入串口
            self.ser.flush()                  # 必须 flush，否则数据滞留缓冲区
            return s
        except Exception as e:                # termios.error/SerialException/ValueError 一网打尽
            self.get_logger().warn(f'串口掉线：{e}，下次自动重连')
            self._force_close()
            return None
    def _read_line(self):
        """读一行回显（半双工，读到 ! 结尾为止）。"""
        if self.ser is None:
            return None
        buf = b''
        t0 = time.time()
        while time.time() - t0 < 0.5:         # 最多等 0.5 秒，避免卡死
            try:
                b = self.ser.read(1)          # 读 1 字节
            except Exception as e:            # USB 抽风时 read 也会炸，不能放它出去
                self.get_logger().warn(f'串口读取失败：{e}，强制重连')
                self._force_close()
                return None
            if b:
                buf += b
                if b == END.encode():         # 读到 ! 就停
                    break
        return buf.decode('ascii', errors='ignore')
    def _read_position(self, sid):
        """发 PRAD 读回实际 PWM 位置。返回 int 或 None。"""
        try:
            with self._ser_lock:                   # 发问和听回答必须一体：防止别的线程插进来，把别人的应答当自己的
                if self.ser is not None:
                    self.ser.reset_input_buffer()  # 丢掉缓冲区里的陈旧残留，防止把上一次的应答错当这一次的
                self._write_cmd(sid, READ_POS)     # 发 #xxxPRAD!
                line = self._read_line()
        except Exception as e:                 # 保险丝：任何异常都消化在这，绝不让它炸穿 _execute_cb
            self.get_logger().warn(f'读取事务异常：{e}，强制重连')
            self._force_close()
            return None
        digits = ''.join(c for c in (line or '') if c.isdigit())   # 只留数字字符
        if not digits:
            return None
        return int(digits) % 10000            # 取后 4 位，避开 ID 位
    def _send_position(self, sid, pwm, duration_ms):
        """统一的底层发送函数：Action 和 /servo/command 订阅都走这里。"""
        try:
            with self._ser_lock:                   # PULR+移动必须整包独占串口，防止两个线程交错发送
                self._write_cmd(sid, RESTORE_TORQUE)   # 动之前必须先 PULR（任务1踩过的坑写进代码）
                time.sleep(0.1)
                # pwm 和 time 都要补成 4 位（0500~2500、0000~9999），否则 <1000 会被拒收
                return self._write_cmd(sid, f'{P}{pwm:04d}{T}{duration_ms:04d}')
        except Exception as e:                 # 保险丝：发送事务异常同样消化在这，不许炸穿 _execute_cb
            self.get_logger().warn(f'发送事务异常：{e}，强制重连')
            self._force_close()
            return None
    def _goal_cb(self, goal_request):
        """发指令之前就校验，不合法直接 REJECT。"""
        cmd = goal_request.command                 # 从 goal 里取出 command（就是那个 msg）
        pwm_min = self.get_parameter('pwm_min').value   # 实时读，不存副本（关键！）
        pwm_max = self.get_parameter('pwm_max').value
        servo_ids = self.get_parameter('servo_ids').value
        if not (pwm_min <= cmd.pwm <= pwm_max):    # 超限 → 拒
            self.get_logger().error(f'goal 被拒：pwm {cmd.pwm} 超限 [{pwm_min},{pwm_max}]')
            return GoalResponse.REJECT
        if cmd.id not in servo_ids:                # ID 不存在 → 拒
            self.get_logger().error(f'goal 被拒：ID {cmd.id} 不在 servo_ids')
            return GoalResponse.REJECT
        if not (0 <= cmd.duration_ms <= 10000):    # duration 非法 → 拒
            self.get_logger().error(f'goal 被拒：duration_ms 非法')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT        
    def _cancel_cb(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT
    def _execute_cb(self, goal_handle):
        cmd = goal_handle.request.command
        sid = cmd.id
        pwm = cmd.pwm
        dur = cmd.duration_ms if cmd.duration_ms > 0 else self.get_parameter('default_duration_ms').value

        start_pwm = self._read_position(sid) or NEUTRAL_PWM   # 读起始位置（用于估算 progress）
        #self._send_position(sid, pwm, dur)                    # 先 PULR 再发位置指令
        self.cmd_pub.publish(ServoCommand(id=sid, pwm=pwm, duration_ms=dur)) 

        fb = MoveServo.Feedback()      # 反馈消息对象
        result = MoveServo.Result()    # 结果消息对象

        step = 0.2                     # 5 Hz（每 200ms 一条 feedback）
        total = dur / 1000.0
        elapsed = 0.0
        while elapsed < total:
            if goal_handle.is_cancel_requested:        # 每轮都检查是否被取消
                self._send_position(sid, NEUTRAL_PWM, 200)   # 取消：回中位
                result.success = False
                result.message = 'canceled'
                result.final_pwm = NEUTRAL_PWM
                goal_handle.canceled()                # 告诉 ROS2：这个 goal 已取消
                return result
            progress = min(1.0, elapsed / total)       # 进度 0~1
            fb.id = sid
            fb.current_pwm = int(start_pwm + (pwm - start_pwm) * progress)   # 按进度线性估算当前 PWM
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)           # 发布 feedback
            time.sleep(step)                           # 睡 200ms（多线程下不阻塞 cancel）
            elapsed += step

        # 走完读回校验：差超 ±30 算没到位，自动重发一次
        time.sleep(0.3)   # 等舵机停稳再读，否则读到"还在动"的中间值
        actual = self._read_position(sid)
        result.final_pwm = actual if actual is not None else 0   # uint16 不能是 -1
        if actual is not None and abs(actual - pwm) <= 30:
            result.success = True
            result.message = 'ok'
        else:
            # 没到位 → 自动重发一次（PRAD 闭环校验加分项）
            self.get_logger().warn(f'未到位：目标 {pwm}，实际 {actual}，自动重发一次')
            self._send_position(sid, pwm, dur)
            time.sleep(dur / 1000.0)
            actual2 = self._read_position(sid)
            result.final_pwm = actual2 if actual2 is not None else 0
            if actual2 is not None and abs(actual2 - pwm) <= 30:
                result.success = True
                result.message = '重发后到位'
                self.get_logger().info(f'到位：重发后实际 {actual2}')
            else:
                result.success = False
                result.message = f'重发后仍未到位：目标 {pwm}，实际 {actual2}'
        goal_handle.succeed()                          # 告诉 ROS2：goal 成功结束
        return result
    def _command_cb(self, msg):
        """订阅 /servo/command，走和 Action 相同的底层发送函数。"""
        self._send_position(msg.id, msg.pwm, msg.duration_ms)
    def _startup_reset(self):
        self.get_logger().info('启动回中位…')
        for sid in self.get_parameter('servo_ids').value:
            self._send_position(sid, NEUTRAL_PWM, 500)
        time.sleep(0.6)
    def _publish_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)                    # 初始化 ROS2（必须最先做）
    node = ServoDriver()
    executor = MultiThreadedExecutor(num_threads=8)   # 明确给足线程！默认=CPU核数，虚拟机可能只有2个，
                                                      # 会被并发的 execute_cb 占满，心跳发不出→看门狗误杀
    executor.add_node(node)
    try:
        executor.spin()                      # 进入事件循环，回调才会被触发
    except KeyboardInterrupt:
        pass                                 # Ctrl+C 优雅退出
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass                                       
if __name__ == '__main__':
    main()        
        
        
        
        
        
        
        
        
                
        
        
