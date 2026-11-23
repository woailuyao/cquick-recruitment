"""Action 客户端最小示例：发一条 goal，等 result 并打印。

验收时评委会直接改用 `ros2 action send_goal`，这个客户端只是给你自己现场 demo 用。
"""
import rclpy
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


def main():
    """1 号舵机 3 秒走到 1800，打印成功/失败原因。"""
    rclpy.init()
    node = rclpy.create_node("servo_client")
    client = ActionClient(node, MoveServo, "move_servo")
    if not client.wait_for_server(timeout_sec=5.0):
        print("等不到 move_servo 服务端，先启动 servo_driver")
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = MoveServo.Goal()
    goal.id = 1
    goal.pwm = 1800
    goal.duration_ms = 3000   # 3 秒慢慢走，留出时间发抢占

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    gh = future.result()
    if gh is None or not gh.accepted:
        print("goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return

    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    print("result:", res_future.result().result)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
