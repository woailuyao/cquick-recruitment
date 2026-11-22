import rclpy
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo
import sys

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_client")
    client = ActionClient(node, MoveServo, "move_servo")

    node.get_logger().info("waiting action server...")
    client.wait_for_server()

    goal = MoveServo.Goal()
    goal.id = 1
    goal.pwm = 1700
    goal.duration_ms = 1000

    future = client.send_goal_async(goal, feedback_callback=lambda fb:
        node.get_logger().info(f"feedback progress={fb.feedback.progress:.2f} curr_pwm={fb.feedback.current_pwm}"))

    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("goal rejected")
        return
    res_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    result = res_future.result().result
    node.get_logger().info(f"result success={result.success} msg={result.message} final_pwm={result.final_pwm}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
