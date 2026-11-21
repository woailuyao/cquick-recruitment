"""move_client.py —— Action 客户端（任务 2 第四步）

按 pwm_list 依次下发 goal，每条等 result 回来再发下一条。
用法：
    ros2 run cquick_servo move_client
    ros2 run cquick_servo move_client --ros-args -p servo_id:=1 -p pwm_list:=[1700,1300,1500]
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')

        self.declare_parameter('servo_id', 1)
        self.declare_parameter('pwm_list', [1700, 1300, 1500])
        self.declare_parameter('duration_ms', 1000)
        self.declare_parameter('step_gap_s', 0.5)

        self._servo_id = int(self.get_parameter('servo_id').value)
        self._duration_ms = int(self.get_parameter('duration_ms').value)
        self._gap = float(self.get_parameter('step_gap_s').value)
        self._queue = [int(p) for p in self.get_parameter('pwm_list').value]

        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info('等待 action server /move_servo ...')
        self._client.wait_for_server()
        self.get_logger().info(
            f'已连接。计划：id={self._servo_id} pwm={self._queue}')
        self._send_next()

    def _send_next(self):
        if not self._queue:
            self.get_logger().info('全部动作完成')
            return

        pwm = self._queue.pop(0)
        goal = MoveServo.Goal()
        goal.id = self._servo_id
        goal.pwm = pwm
        goal.duration_ms = self._duration_ms

        self.get_logger().info(f'发送 goal：pwm={pwm} duration={self._duration_ms}')
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'feedback: id={self._servo_id} pwm={feedback.current_pwm} '
            f'progress={feedback.progress:.2f}')

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（检查 pwm 是否在安全区间、id 是否存在）')
            self._schedule_next()
            return
        self.get_logger().info('goal 已接受，等待 result ...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'result: 成功，{result.message}')
        else:
            self.get_logger().warning(f'result: 失败，{result.message}')
        self._schedule_next()

    def _schedule_next(self):
        # 用一次性定时器等一会儿再发下一条，避免在回调里直接递归
        self._next_timer = self.create_timer(self._gap, self._next_tick)

    def _next_tick(self):
        self._next_timer.cancel()
        self._send_next()


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
