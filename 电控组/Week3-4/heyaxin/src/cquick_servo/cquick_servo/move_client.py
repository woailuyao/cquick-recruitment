"""
move_client.py — Action 客户端（演示用）

评审一般直接用 `ros2 action send_goal` 打 goal。
这个客户端的作用是演示完整链路：发 goal、逐条打印 feedback、打印 result。

用法：
    ros2 run cquick_servo move_client [id] [pwm] [duration_ms]
    默认 0 1500 1500（本套硬件单个舵机 ID 为 0）
"""

import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')
        self._last_servo_id = 0
        self._finished = threading.Event()

    def send_goal(self, servo_id, pwm, duration_ms):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('等不到 /move_servo 服务端，先启动 servo_driver')
            return False

        self._last_servo_id = servo_id

        goal = MoveServo.Goal()
        goal.command = ServoCommand(id=servo_id, pwm=pwm, duration_ms=duration_ms)

        self.get_logger().info(
            f'发送 goal: id={servo_id} pwm={pwm} duration={duration_ms}')
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 被拒绝（ID 不在 servo_ids / pwm 超限位 / duration 非法）')
            self._finished.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 输出格式固定为：id=0 pwm=1620 progress=0.62
        # flush：stdout 与 logger 走不同流，不刷新会让 result 抢在 feedback 前打印
        print(f'id={self._last_servo_id} pwm={feedback.current_pwm} '
              f'progress={feedback.progress:.2f}', flush=True)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'成功: final_pwm={result.final_pwm} — {result.message}')
        else:
            self.get_logger().warn(f'失败: {result.message}')
        self._finished.set()

    def wait_until_finished(self):
        """收到 result（或 goal 被拒）后返回，让命令行调用也能自然结束。"""
        while rclpy.ok() and not self._finished.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()

    # launch 会在命令行后面追加 `--ros-args -r __node:=...`，
    # 直接取 sys.argv[1] 会把 '--ros-args' 当成舵机 ID 解析而崩掉。
    # remove_ros_args 把 ROS 自己的参数剪掉，只留 id / pwm / duration_ms。
    positional = remove_ros_args(args=sys.argv)[1:]
    try:
        servo_id = int(positional[0]) if len(positional) > 0 else 0
        pwm = int(positional[1]) if len(positional) > 1 else 1500
        duration_ms = int(positional[2]) if len(positional) > 2 else 1500
    except ValueError:
        node.get_logger().error(
            f'参数必须都是整数：id pwm duration_ms，收到 {positional}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    try:
        if node.send_goal(servo_id, pwm, duration_ms):
            node.wait_until_finished()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
