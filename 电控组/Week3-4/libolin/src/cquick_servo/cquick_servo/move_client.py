#!/usr/bin/env python3
import sys
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo

class ServoMoveClient(Node):
    def __init__(self):
        super().__init__('servo_move_client')
        self.cli = ActionClient(self, MoveServo, '/move_servo')
        self.goal_handle = None
        # 注册Ctrl+C信号捕获
        signal.signal(signal.SIGINT, self.sigint_handler)

    def send_goal(self, sid, pwm, duration):
        self.cli.wait_for_server()
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration
        self.get_logger().info(f"发送目标: id={sid}, pwm={pwm}, duration={duration}ms")
        send_future = self.cli.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_resp_cb)

    def goal_resp_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("目标被拒绝")
            return
        self.get_logger().info("目标已接受，开始运动")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(f"实时状态: current_pwm={fb.current_pwm:.0f}, progress={fb.progress:.2f}")

    def result_cb(self, future):
        res = future.result().result
        self.get_logger().info(f"动作结束: success={res.success}, message={res.message}, final_pwm={res.final_pwm}")
        rclpy.shutdown()

    def sigint_handler(self, signum, frame):
        self.get_logger().info("收到Ctrl+C，正在发送取消请求...")
        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_cb)

    def cancel_done_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("取消成功，舵机正在回中位")
        else:
            self.get_logger().info("取消失败，目标已经结束")
            rclpy.shutdown()

def main():
    rclpy.init()
    # 支持命令行传参：ros2 run cquick_servo move_client [id] [pwm] [时长ms]
    sid = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 1700
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 3000
    node = ServoMoveClient()
    node.send_goal(sid, pwm, duration)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

