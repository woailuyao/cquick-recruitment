import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


# ============================================================
# 一个 Step = 一步动作组：包含若干舵机的并发命令
# ============================================================
class Step:
    def __init__(self, name, commands):
        self.name = name
        self.commands = commands        # [(servo_id, pwm, duration_ms), ...]

        self.results = []               # 收集每个子 goal 的 result
        self.pending = len(commands)    # 未完成的子 goal 计数
        self.all_success = True         # 本步所有子 goal 是否全部成功
        self.is_done = False            # 本步是否已收尾（防重复触发）

    def add_result(self, servo_id, successful, reason, prad, status):
        self.results.append({
            'servo_id': servo_id,
            'successful': successful,
            'reason': reason,
            'prad': prad,
            'status': status,
        })
        if not successful:
            self.all_success = False
        self.pending -= 1

    def summary(self):
        lines = [
            f"[STEP {self.name}] 本步统一结果 all_success={self.all_success} "
            f"(共 {len(self.results)} 个舵机)"
        ]
        for r in self.results:
            lines.append(
                f"    servo={r['servo_id']} success={r['successful']} "
                f"reason={r['reason']} prad={r['prad']} status={r['status']}"
            )
        return "\n".join(lines)


# ============================================================
# Action 客户端
# ============================================================
class ServoClient(Node):
    def __init__(self, node_name="move_client", stop_on_failure=False):
        super().__init__(node_name)
        self.action_client_ = ActionClient(self, MoveServo, "move_servo")

        self.steps = []                  # 全部步骤
        self.stop_on_failure = stop_on_failure

        self.current_step_idx = -1
        self._current_step = None
        self._goal_handles = []          # 当前 step 所有 goal_handle（用于取消）

    # ---------------- 组装步骤 ----------------
    def add_step(self, name, commands):
        """commands: [(servo_id, pwm, duration_ms), ...]"""
        self.steps.append(Step(name, commands))

    # ---------------- 启动 ----------------
    def start(self):
        self.get_logger().info("等待 action server ...")
        self.action_client_.wait_for_server()   # 只在 spin 之前等一次
        self._run_step(0)

    # ========================================================
    # 核心：执行某一步
    # ========================================================
    def _run_step(self, idx):
        if idx >= len(self.steps):
            self.get_logger().info("===== 所有动作组执行完毕 =====")
            return

        step = self.steps[idx]
        self.current_step_idx = idx
        self._current_step = step
        self._goal_handles = []

        self.get_logger().info(
            f"\n=== 开始 Step[{idx}] {step.name}，"
            f"共 {len(step.commands)} 个舵机命令（并发发送） ===")

        if step.pending == 0:                # 空步骤直接跳过
            self._on_step_complete(idx)
            return

        # 并发发出所有 goal（send_goal_async 立即返回，不阻塞）
        for (sid, pwm, dur) in step.commands:
            self._send_one_goal(step, idx, sid, pwm, dur)

    def _send_one_goal(self, step, step_idx, sid, pwm, dur):
        goal = MoveServo.Goal()
        goal.command.id = sid
        goal.command.pwm = pwm
        goal.command.duration_ms = dur

        self.get_logger().info(f"  → 发送 servo={sid} pwm={pwm} dur={dur}ms")

        send_future = self.action_client_.send_goal_async(
            goal, feedback_callback=self._make_feedback_cb(sid)
        )
        # 注意：用默认参数绑定 sid/step/step_idx，避免 lambda 延迟绑定 bug
        send_future.add_done_callback(
            lambda fut, _s=step, _i=step_idx, _sid=sid:
                self._on_goal_response(_s, _i, _sid, fut)
        )

    def _make_feedback_cb(self, sid):
        def cb(feedback_msg):
            fb = feedback_msg.feedback
            self.get_logger().info(
                f"  [FB] servo={sid} pwm={fb.pwm} progress={fb.progress}")
        return cb

    # ---------------- Goal 响应 ----------------
    def _on_goal_response(self, step, step_idx, sid, future):
        if step.is_done:                     # 已收尾，忽略迟到回调
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"servo={sid} 发送失败: {e}")
            step.add_result(sid, False, f"send_goal异常:{e}", None, None)
            self._check_step_complete(step, step_idx)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"servo={sid} 目标被拒绝")
            step.add_result(sid, False, "rejected", None, None)
            self._check_step_complete(step, step_idx)
            return

        self._goal_handles.append(goal_handle)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, _s=step, _i=step_idx, _sid=sid:
                self._on_result(_s, _i, _sid, fut)
        )

    # ---------------- Result 回调 ----------------
    def _on_result(self, step, step_idx, sid, future):
        if step.is_done:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
            step.add_result(
                sid, result.successful, result.reason, result.prad, status
            )
        except Exception as e:
            self.get_logger().warn(f"servo={sid} 获取结果异常: {e}")
            step.add_result(sid, False, f"result异常:{e}", None, None)

        self._check_step_complete(step, step_idx)

    # ---------------- 判断本步是否收齐 ----------------
    def _check_step_complete(self, step, step_idx):
        if step.is_done:
            return
        if step.pending == 0:
            self._on_step_complete(step_idx)

    def _on_step_complete(self, step_idx):
        step = self.steps[step_idx]
        if step.is_done:
            return
        step.is_done = True

        # ★ 每步统一打印本步的 result 汇总
        self.get_logger().info(step.summary())

        # 失败策略：是否终止后续步骤
        if self.stop_on_failure and not step.all_success:
            self.get_logger().warn(
                f"Step[{step_idx}] 存在失败子任务，stop_on_failure=True，终止")
            return

        # ★ 本步全部完成，立即执行下一步
        self._run_step(step_idx + 1)

    # ---------------- 取消当前 step ----------------
    def cancel_current_step(self):
        step = self._current_step
        if step is None or step.is_done:
            return
        self.get_logger().warn(f"取消 Step[{self.current_step_idx}] 的全部目标")
        for gh in self._goal_handles:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"取消失败: {e}")


# ============================================================
def main():
    rclpy.init()
    node = ServoClient(stop_on_failure=False)

    # 一步 = 同一时刻给多个舵机并行发命令
    node.add_step("回中", [
        (0, 1500, 1000),
        (1, 1500, 1000),
        (2, 1500, 1000),
    ])
    node.add_step("同步展开", [
        (0, 2000, 2000),
        (1, 2000, 2000),
        (2, 2000, 2000),
    ])
    node.add_step("再回中", [
        (0, 1500, 1000),
        (1, 1500, 1000),
        (2, 1500, 1000),
    ])

    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()