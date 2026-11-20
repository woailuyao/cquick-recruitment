#!/usr/bin/env python3
"""任务3逐关节自检：校准中位 → 逻辑 +delta → 校准中位。

用法：
    ros2 run cquick_servo joint_selfcheck
    ros2 run cquick_servo joint_selfcheck --ros-args -p delta:=50 -p duration_ms:=1200

每颗舵机的三个阶段都等待 Action result，不用 sleep 判断到位。
阶段间 pause_s 只用于观察，不参与动作同步。任一失败/超时立即整机卸力。
"""
import time

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from cquick_servo_interfaces.action import MoveServo


class JointSelfcheck(Node):
    def __init__(self):
        super().__init__('joint_selfcheck')
        self.declare_parameter('delta', 50)
        self.declare_parameter('duration_ms', 1200)
        self.declare_parameter('pause_s', 0.5)
        self._action = ActionClient(self, MoveServo, '/move_servo')
        self._get_params = self.create_client(
            GetParameters, '/servo_driver/get_parameters')
        self._set_params = self.create_client(
            SetParameters, '/servo_driver/set_parameters')
        self._active_handle = None
        self._emergency_sent = False

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('等待 ROS2 响应超时')
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result()

    def _driver_config(self):
        if not self._get_params.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('驱动参数服务不可用')
        request = GetParameters.Request()
        request.names = ['servo_ids', 'joint_offsets', 'home_pwm']
        response = self._wait(self._get_params.call_async(request), 5.0)
        values = response.values
        return (
            [int(value) for value in values[0].integer_array_value],
            [int(value) for value in values[1].integer_array_value],
            int(values[2].integer_value),
        )

    def emergency_stop(self):
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()
        if self._emergency_sent:
            return
        self._emergency_sent = True
        if self._set_params.service_is_ready():
            request = SetParameters.Request()
            request.parameters = [
                Parameter('torque_enable', value=False).to_parameter_msg()]
            self._set_params.call_async(request)
        self.get_logger().error('自检异常：已请求整机卸力，禁止继续测试下一颗')

    def send_goal(self, servo_id, logical_pwm, duration_ms, applied_pwm):
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(logical_pwm)
        goal.duration_ms = int(duration_ms)
        self.get_logger().info(
            f'id={servo_id} logical={logical_pwm} applied={applied_pwm} '
            f'duration={duration_ms}ms')

        handle = self._wait(self._action.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(f'id={servo_id} goal 被拒绝')
        self._active_handle = handle
        # 超时时保留 handle，emergency_stop 才能继续取消它。
        response = self._wait(
            handle.get_result_async(), duration_ms / 1000.0 + 5.0)
        self._active_handle = None
        result = response.result
        self.get_logger().info(
            f'id={servo_id} result success={result.success} '
            f'message={result.message} final_pwm={result.final_pwm}')
        if not result.success:
            raise RuntimeError(f'id={servo_id} 未正常到位: {result.message}')
        return int(result.final_pwm)

    def run(self):
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('move_servo 不可用')
        servo_ids, offsets, home = self._driver_config()
        delta = int(self.get_parameter('delta').value)
        duration = int(self.get_parameter('duration_ms').value)
        pause = float(self.get_parameter('pause_s').value)
        if not 0 <= delta <= 100:
            raise ValueError('自检 delta 只允许 0~100，正式限位测试另做')
        if not 1 <= duration <= 10000:
            raise ValueError('duration_ms 必须在 1~10000')

        self.get_logger().info(
            f'开始逐关节自检 ids={servo_ids}，每颗：{home} → {home + delta} → {home}')
        for servo_id in servo_ids:
            index = servo_id - 1
            if not 0 <= index < len(offsets):
                raise RuntimeError(f'id={servo_id} 缺少 joint_offset')
            offset = offsets[index]
            self.get_logger().info(f'========== 测试 ID {servo_id} ==========')
            readings = []
            for logical in (home, home + delta, home):
                applied = logical + offset
                readings.append(self.send_goal(
                    servo_id, logical, duration, applied))
                time.sleep(pause)  # 仅留观察间隔；到位已由 result 确认
            self.get_logger().info(
                f'ID {servo_id} 完成，读回={readings}，'
                f'请核对 +delta 时的机械方向')

        self.get_logger().info('全部关节自检成功，均已回到校准标准位')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = JointSelfcheck()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('Ctrl+C，停止自检')
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        if not ok:
            node.emergency_stop()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
