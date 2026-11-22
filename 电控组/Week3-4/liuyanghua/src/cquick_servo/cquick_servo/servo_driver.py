import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import math
import threading
import functools
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


class ServoServer(Node):
    def __init__(self, node_name="servo_driver"):
        super().__init__(node_name)

        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.action_server_ = ActionServer(
            self, MoveServo, "move_servo",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group,
        )

        self.declare_parameter('port', '滚木')
        self.declare_parameter('baudrate', 0)
        self.declare_parameter('pwm_min', 0)
        self.declare_parameter('pwm_max', 0)
        self.declare_parameter('default_duration_ms', 0)
        self.declare_parameter('servo_ids', [0])
        self.declare_parameter('startup_reset', True)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.default_duration_ms = self.get_parameter('default_duration_ms').value
        self.servo_ids = self.get_parameter('servo_ids').value
        self.startup_reset = self.get_parameter('startup_reset').value

        # 串口
        self.ser = None
        self.ser_lock = threading.Lock()
        self.ser_ref = 0

        # 共享状态（Condition 用于跨线程等待/唤醒）
        self.state_cond = threading.Condition()
        self.active_ids = set()                       # 正在执行的舵机 id
        self.active_goals = {}                        # key -> threading.Event
        self.goal_by_id = {}                          # id -> key（同一时刻最多 1 个）
        self.preempt_flags = {}                       # key -> bool（是否被抢占，检测旧action是否为抢占结束分支）
        self.feedback_timers = {}                     # key -> timer
        self.servo_positions = {}                     # id -> 当前位置
        self.action_running_count = 0

        self.state_pub = self.create_publisher(ServoCommand, "/servo/state", 10)
        self.command_pub = self.create_publisher(ServoCommand, "/servo/command", 10)
        self.subscriber_ = self.create_subscription(
            ServoCommand, "/servo/command", self.command_callback, 10)

        self.add_on_set_parameters_callback(self.parameter_callback)

        if self.startup_reset:
            self.startup_reset_func()

    # ------------------------------------------------------------------
    # 参数回调
    # ------------------------------------------------------------------
    def parameter_callback(self, params):
        bdr = [9600, 115200]
        for param in params:
            if param.name == 'baudrate':
                if param.value not in bdr:
                    self.get_logger().warn(f"参数{param.name}超出指定范围{bdr}")
                    return SetParametersResult(successful=False, reason=f"参数{param.name}超出指定范围{bdr}")
            elif param.name == 'pwm_min':
                if param.value > self.pwm_max or param.value < 500 or param.value > 2500:
                    if param.value < 500 or param.value > 2500:
                        self.get_logger().warn(f"参数{param.name}超出指定范围500~2500")
                        return SetParametersResult(successful=False, reason=f"参数{param.name}超出指定范围500~2500")
                    self.get_logger().warn(f"参数{param.name}大于pwm_max")
                    return SetParametersResult(successful=False, reason=f"参数{param.name}大于pwm_max")
            elif param.name == 'pwm_max':
                if param.value < self.pwm_min or param.value < 500 or param.value > 2500:
                    if param.value < 500 or param.value > 2500:
                        self.get_logger().warn(f"参数{param.name}超出指定范围500~2500")
                        return SetParametersResult(successful=False, reason=f"参数{param.name}超出指定范围500~2500")
                    self.get_logger().warn(f"参数{param.name}小于pwm_min")
                    return SetParametersResult(successful=False, reason=f"参数{param.name}小于pwm_min")
            elif param.name == 'default_duration_ms':
                if param.value < 0 or param.value > 9999:
                    self.get_logger().warn(f"参数{param.name}超出指定范围0~9999")
                    return SetParametersResult(successful=False, reason=f"参数{param.name}超出指定范围0~9999")
            elif param.name == 'servo_ids':
                for _id in param.value:
                    if _id < 0 or _id > 255:
                        self.get_logger().warn(f"参数{param.name}中存在元素超出指定范围0~255")
                        return SetParametersResult(successful=False, reason=f"参数{param.name}中存在元素超出指定范围0~255")
            self.get_logger().info(f"参数{param.name}合法")

        for param in params:
            if param.name == 'port':
                self.port = param.value
            elif param.name == 'baudrate':
                self.baudrate = param.value
            elif param.name == 'pwm_min':
                self.pwm_min = param.value
            elif param.name == 'pwm_max':
                self.pwm_max = param.value
            elif param.name == 'default_duration_ms':
                self.default_duration_ms = param.value
            elif param.name == 'servo_ids':
                self.servo_ids = param.value
            elif param.name == 'startup_reset':
                self.startup_reset = param.value
            self.get_logger().info(f"参数{param.name}被修改为{param.value}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # 串口引用计数（**不**访问 state_cond）
    # ------------------------------------------------------------------
    def _ensure_serial_open_locked(self):
        if self.ser is not None and self.ser.is_open:
            self.ser_ref += 1
            return True
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser_ref = 1
            return True
        except Exception as e:
            self.get_logger().error(f"打开串口失败: {e}")
            self.ser = None
            self.ser_ref = 0
            return False

    def _release_serial_locked(self):
        if self.ser_ref > 0:
            self.ser_ref -= 1
        if self.ser_ref == 0 and self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.ser = None

    def _release_serial(self):
        with self.ser_lock:
            self._release_serial_locked()

    # ------------------------------------------------------------------
    # 计数
    # ------------------------------------------------------------------
    def _inc_running(self):
        with self.state_cond:
            self.action_running_count += 1

    def _dec_running(self):
        with self.state_cond:
            self.action_running_count -= 1

    # ------------------------------------------------------------------
    # 抢占停止指令
    # ------------------------------------------------------------------
    def _send_pdst(self, sid):
        with self.ser_lock:
            if self.ser is None or not self.ser.is_open:
                self.get_logger().warn(f"[servo:{sid}] 抢占 PDST 未发送：串口未打开")
                return False
            try:
                self.ser.reset_input_buffer()
                self.ser.write(f"#{sid:03d}PDST!".encode("ascii"))
                self.ser.flush()
                self.get_logger().warn(f"[servo:{sid}] 发送抢占停止指令：#{sid:03d}PDST!")
                return True
            except Exception as e:
                self.get_logger().error(f"[servo:{sid}] 抢占 PDST 发送失败: {e}")
                return False

    # ------------------------------------------------------------------
    # 启动回中
    # ------------------------------------------------------------------
    def startup_reset_func(self):
        cmd = ServoCommand()
        cmd.id = 255
        cmd.pwm = 1500
        cmd.duration_ms = 1000
        try:
            with self.ser_lock:
                if self.ser is None or not self.ser.is_open:
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.ser.reset_input_buffer()
                self.get_logger().info(f"启动时回中位：上力，发送：#{cmd.id:03d}PULR!")
                self.ser.write(f"#{cmd.id:03d}PULR!".encode("ascii"))
                self.ser.flush()
                self.get_logger().info(
                    f"启动时回中位：发送运动指令：#{cmd.id:03d}P{cmd.pwm:04d}T{cmd.duration_ms:04d}!")
                self.ser.write(
                    f"#{cmd.id:03d}P{cmd.pwm:04d}T{cmd.duration_ms:04d}!".encode("ascii"))
                self.ser.flush()
            return True
        except Exception as e:
            self.get_logger().error(f"启动时回中位：串口操作失败: {e}")
            return False

    # ------------------------------------------------------------------
    # 底层发送：ser_lock 内不触碰 state_cond；state_cond 内不碰串口
    # ------------------------------------------------------------------
    def send_hardware_command_(self, cmd):
        init_position = None
        response_str = ""
        with self.ser_lock:
            if not self._ensure_serial_open_locked():
                return False, -31, "串口打开失败"
            try:
                self.ser.reset_input_buffer()
                self.ser.write(f"#{cmd.id:03d}PULR!".encode("ascii"))
                self.ser.flush()

                self.ser.reset_input_buffer()
                self.ser.write(f"#{cmd.id:03d}PRAD!".encode("ascii"))
                self.ser.flush()
                response_str = self.ser.read(10).decode('ascii').strip()

                self.get_logger().info(
                    f"发送运动指令：#{cmd.id:03d}P{cmd.pwm:04d}T{cmd.duration_ms:04d}!")
                self.ser.write(
                    f"#{cmd.id:03d}P{cmd.pwm:04d}T{cmd.duration_ms:04d}!".encode("ascii"))
                self.ser.flush()
            except Exception as e:
                self._release_serial_locked()
                self.get_logger().error(f"串口操作失败: {e}")
                return False, -31, f"串口操作失败: {e}"

            if len(response_str) == 10 and response_str[0] == '#' and response_str[9] == '!':
                init_position = int(response_str[5:9])
            else:
                self._release_serial_locked()
                self.get_logger().warn(f"读取舵机初始位置失败: {response_str}")
                return False, -31, "读取舵机初始位置失败"

        # 锁外更新共享位置
        with self.state_cond:
            self.servo_positions[cmd.id] = init_position
        self.get_logger().info(f"初始位置：{init_position}")
        return True, init_position, response_str

    # ------------------------------------------------------------------
    # Topic 回放
    # ------------------------------------------------------------------
    def command_callback(self, msg):
        with self.state_cond:
            running = self.action_running_count > 0
        if running:
            self.get_logger().warn(f"[servo:{msg.id}] TEST: 回放被拦截")
            return

        self.get_logger().info("/servo/command Topic 回放中...")
        success, _, reason = self.send_hardware_command_(msg)
        if not success:
            self.get_logger().error(f"回放指令执行失败: {reason}")
            return

        current_pos = None
        try:
            with self.ser_lock:
                if self.ser and self.ser.is_open:
                    self.ser.reset_input_buffer()
                    self.ser.write(f"#{msg.id:03d}PRAD!".encode("ascii"))
                    self.ser.flush()
                    response_str = self.ser.read(10).decode('ascii').strip()
                    if len(response_str) == 10:
                        current_pos = int(response_str[5:9])
        except Exception:
            pass
        finally:
            self._release_serial()

        if current_pos is not None:
            with self.state_cond:
                self.servo_positions[msg.id] = current_pos
            state_msg = ServoCommand()
            state_msg.id = msg.id
            state_msg.pwm = current_pos
            try:
                self.state_pub.publish(state_msg)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Action 执行回调（含抢占）
    # ------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        result = MoveServo.Result()
        key = tuple(goal_handle.goal_id.uuid)

        self._inc_running()
        serial_held = False
        registered = False
        try:
            self.get_logger().info(f"[servo:{cmd.id}] 收到动作请求 (key={key[:4]})")

            # ============ 阶段 1：抢占同 id 的旧 goal ============
            preempt_old_key = None
            with self.state_cond:
                if cmd.id in self.active_ids:
                    preempt_old_key = self.goal_by_id.get(cmd.id)
                    if preempt_old_key is not None:
                        self.preempt_flags[preempt_old_key] = True
                        old_ev = self.active_goals.get(preempt_old_key)
                        if old_ev is not None:
                            old_ev.set() #停掉旧goal

            if preempt_old_key is not None:
                self.get_logger().warn(
                    f"[servo:{cmd.id}] 检测到已有 action，执行抢占 (old_key={preempt_old_key[:4]})")
                self._send_pdst(cmd.id)
                with self.state_cond:
                    released = self.state_cond.wait_for(
                        lambda: cmd.id not in self.active_ids,
                        timeout=1.0)
                if not released:
                    self.get_logger().error(
                        f"[servo:{cmd.id}] 抢占超时：旧 action 未在 1s 内退出")
                    goal_handle.abort()
                    result.successful = False
                    result.reason = f"[servo:{cmd.id}] 抢占超时"
                    with self.state_cond:
                        result.prad = int(self.servo_positions.get(cmd.id, 0))
                    return result

            # ============ 阶段 2：注册自己 ============
            with self.state_cond:
                if cmd.id in self.active_ids:
                    # 极端竞态：另一路抢先注册
                    self.get_logger().warn(
                        f"[servo:{cmd.id}] 注册失败：设备仍被占用")
                    goal_handle.abort()
                    result.successful = False
                    result.reason = f"[servo:{cmd.id}] 注册失败：设备被占用"
                    result.prad = int(self.servo_positions.get(cmd.id, 0))
                    return result
                self.active_ids.add(cmd.id)
                finish_event = threading.Event()
                self.active_goals[key] = finish_event
                self.goal_by_id[cmd.id] = key
                self.preempt_flags[key] = False
                registered = True

            # ============ 阶段 3：正常执行 ============
            feedback_msg = MoveServo.Feedback()
            timeout_sec = max(cmd.duration_ms / 1000.0 * 1.1, 0.04)

            self.command_pub.publish(cmd)

            success, init_position, reason = self.send_hardware_command_(cmd)
            if not success:
                result.prad = -31
                result.successful = False
                result.reason = reason
                return result
            serial_held = True

            init_rad_distance = init_position - cmd.pwm

            timer = self.create_timer(
                0.05,
                functools.partial(
                    self.feedback_timer_callback,
                    feedback_msg=feedback_msg,
                    goal_handle=goal_handle,
                    init_rad_distance=init_rad_distance,
                    init_position=init_position,
                    cmd=cmd,
                    event=finish_event,
                ),
                callback_group=self.timer_cb_group,
            )
            with self.state_cond:
                self.feedback_timers[key] = timer

            finish_event.wait(timeout=timeout_sec)

            self._cancel_timer_only(key)

            with self.state_cond:
                was_preempted = self.preempt_flags.get(key, False)

            if goal_handle.is_cancel_requested:
                with self.ser_lock:
                    if self.ser and self.ser.is_open:
                        self.get_logger().warn(f"[servo:{cmd.id}] 任务已取消")
                        self.ser.reset_input_buffer()
                        self.ser.write(f"#{cmd.id:03d}PDST!".encode("ascii"))
                        self.ser.flush()
                        self.ser.reset_input_buffer()
                        self.ser.write(f"#{cmd.id:03d}P1500T1000!".encode("ascii"))
                        self.ser.flush()
                goal_handle.canceled()
                result.successful = False
                result.reason = "canceled"
                with self.state_cond:
                    result.prad = int(self.servo_positions.get(cmd.id, 0))
                return result

            if was_preempted: #抢占分支
                self.get_logger().warn(
                    f"[servo:{cmd.id}] action 被新 goal 抢占，abort")
                goal_handle.abort()
                result.successful = False
                result.reason = f"[servo:{cmd.id}] 被新 goal 抢占"
                with self.state_cond:
                    result.prad = int(self.servo_positions.get(cmd.id, 0))
                return result

            # 正常完成
            with self.state_cond:
                current_pos = self.servo_positions.get(cmd.id, 0)
            result.prad = int(current_pos)
            if math.fabs(cmd.pwm - current_pos) <= 30:
                goal_handle.succeed()
                result.successful = True
                result.reason = f"[servo:{cmd.id}] 成功执行动作"
            else:
                goal_handle.abort()
                result.successful = False
                result.reason = (f"[servo:{cmd.id}] 未成功运动到指定位置, "
                                 f"pwm相差: {math.fabs(cmd.pwm - current_pos):.2f}")
            return result

        finally:
            self._cancel_timer_only(key)
            with self.state_cond:
                if registered:
                    self.active_ids.discard(cmd.id)
                    self.active_goals.pop(key, None)
                    if self.goal_by_id.get(cmd.id) == key:
                        self.goal_by_id.pop(cmd.id, None)
                self.preempt_flags.pop(key, None)
                self.state_cond.notify_all()      # 唤醒抢占等待者
            if serial_held:
                self._release_serial()
            self._dec_running()

    # ------------------------------------------------------------------
    # 反馈 timer 回调（加终态守卫）
    # ------------------------------------------------------------------
    def feedback_timer_callback(self, feedback_msg, goal_handle,
                                init_rad_distance, init_position, cmd, event):
        if not goal_handle.is_active:
            event.set()
            return
        if goal_handle.is_cancel_requested:
            event.set()
            return

        try:
            with self.ser_lock:
                if self.ser is None or not self.ser.is_open:
                    return
                self.ser.reset_input_buffer()
                self.ser.write(f"#{cmd.id:03d}PRAD!".encode("ascii"))
                self.ser.flush()
                response_str = self.ser.read(10).decode('ascii').strip()
        except Exception:
            return

        if len(response_str) == 10 and response_str[0] == '#' and response_str[9] == '!':
            position = int(response_str[5:9])
            feedback_msg.pwm = position
            with self.state_cond:
                self.servo_positions[cmd.id] = position

            state_msg = ServoCommand()
            state_msg.id = cmd.id
            state_msg.pwm = position
            try:
                self.state_pub.publish(state_msg)
            except Exception:
                pass

            if init_rad_distance != 0:
                feedback_msg.progress = math.fabs((position - init_position) / init_rad_distance)
            else:
                feedback_msg.progress = 1.0

            if math.fabs(position - cmd.pwm) <= 30:
                event.set()
        else:
            with self.state_cond:
                last = self.servo_positions.get(cmd.id, 0)
            feedback_msg.pwm = last

        if goal_handle.is_active:
            try:
                goal_handle.publish_feedback(feedback_msg)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 只 cancel 不 destroy
    # ------------------------------------------------------------------
    def _cancel_timer_only(self, key):
        with self.state_cond:
            t = self.feedback_timers.pop(key, None)
        if t is not None:
            try:
                t.cancel()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # goal_callback：不再拒绝同 id 新 goal（交给 execute_callback 抢占）
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        cmd = goal_request.command
        if cmd.id not in self.servo_ids:
            self.get_logger().warn(f"[servo:{cmd.id}] 目标ID设备不存在")
            return GoalResponse.REJECT
        if not (self.pwm_min <= cmd.pwm <= self.pwm_max):
            self.get_logger().warn(f"[servo:{cmd.id}] 目标PWM超出限定范围")
            return GoalResponse.REJECT
        if not (0 <= cmd.duration_ms <= 9999):
            self.get_logger().warn(f"[servo:{cmd.id}] 非法duration_ms")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------------
    # cancel_callback
    # ------------------------------------------------------------------
    def cancel_callback(self, goal_handle):
        key = tuple(goal_handle.goal_id.uuid)
        with self.state_cond:
            event = self.active_goals.get(key)
        if event is not None:
            event.set()
        self.get_logger().info(
            f"[servo:{goal_handle.request.command.id}] 收到取消请求，正在取消...")
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = ServoServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()