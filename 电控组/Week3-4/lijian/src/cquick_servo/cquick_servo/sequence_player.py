import sys
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self):
        super().__init__("sequence_player")
        self._client = ActionClient(self, MoveServo, "move_servo")

    def play_yaml(self, yaml_path):
        self.get_logger().info(f"开始播放动作: {yaml_path}")
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server 不在线，请先启动 servo_driver！")
            return

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        steps = data.get("steps", [])
        for i, step in enumerate(steps):
            step_name = step.get("step_name", f"步骤 {i+1}")
            actions = step.get("actions", [])
            self.get_logger().info(f"--- 正在执行: {step_name} ---")

            goal_futures = []
            result_futures = []

            # 1. 遍历当前 step 的所有 action，全部异步发出去
            for act in actions:
                goal = MoveServo.Goal()
                goal.id = act['id']
                goal.pwm = act['pwm']
                goal.duration_ms = act['duration_ms']
                
                # 非阻塞发送
                send_future = self._client.send_goal_async(goal)
                goal_futures.append((act['id'], send_future))

            # 2. 统一等待所有 goal 被 server 接受
            for sid, f in goal_futures:
                rclpy.spin_until_future_complete(self, f)
                goal_handle = f.result()
                if goal_handle.accepted:
                    # 开始等 result
                    result_futures.append((sid, goal_handle.get_result_async()))
                else:
                    self.get_logger().error(f"id={sid} 的 goal 被拒绝")

            # 3. 统一等待所有 result 返回（这一步也保证同 step 内所有舵机都做完才进下一步）
            for sid, f in result_futures:
                rclpy.spin_until_future_complete(self, f)
                result = f.result().result
                self.get_logger().info(
                    f"[{time.strftime('%H:%M:%S')}] id={sid} result: success={result.success}, msg={result.message}")

            time.sleep(0.2)  # 步与步之间微小的停顿

        self.get_logger().info("全部动作播放完毕！")


def main():
    rclpy.init()
    node = SequencePlayer()
    if len(sys.argv) < 2:
        node.get_logger().error("用法: ros2 run cquick_servo sequence_player <yaml_path>")
        return

    node.play_yaml(sys.argv[1])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
