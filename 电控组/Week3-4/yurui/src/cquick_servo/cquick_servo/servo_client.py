import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo
import time

class MoveClient(Node):

    def __init__(self):
        super().__init__("move_client")
        self.client = ActionClient(self, MoveServo, "move_servo")
        self.get_logger().info("客户端起来了，正在找服务器……")

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"进度 {fb.progress:.0%}  当前PWM {fb.current_pwm}")

    def send_goal(self, sid, pwm, dur):
        goal = MoveServo.Goal()
        goal.command.id = sid
        goal.command.pwm = pwm
        goal.command.duration_ms = dur

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("找不到服务器，servo_driver 起了吗？")
            return None

        goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("目标被拒了（越界或ID不存在)")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main():
    rclpy.init()
    node = MoveClient()

    tests = [
        (1, 1500, 1000),
        (2, 1000, 800),
        (1, 2000, 1000),
        (1, 9999, 500),
    ]

    for sid, pwm, dur in tests:
        node.get_logger().info(f"目标：舵机{sid} → PWM {pwm}，{dur}ms ")
        result = node.send_goal(sid, pwm, dur)
        if result is None:
            continue
        node.get_logger().info(
            f"结果：成功={result.success}  最终PWM={result.final_pwm}  原因={result.reason}")
            
    node.get_logger().info("演示取消")
    goal = MoveServo.Goal()
    goal.command.id = 1
    goal.command.pwm = 1500
    goal.command.duration_ms = 3000

    if node.client.wait_for_server(timeout_sec=5.0):
        goal_future = node.client.send_goal_async(goal, feedback_callback=node.feedback_callback)
        rclpy.spin_until_future_complete(node, goal_future)
        gh = goal_future.result()
        if gh.accepted:
            time.sleep(1.0)
            cancel_fut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_fut)
            res_fut = gh.get_result_async()
            rclpy.spin_until_future_complete(node, res_fut)
            r = res_fut.result().result
            node.get_logger().info(f"取消结果：成功={r.success}  原因={r.reason}")


    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

