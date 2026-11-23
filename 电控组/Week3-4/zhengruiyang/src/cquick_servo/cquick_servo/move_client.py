import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient                    # 客户端用 ActionClient
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')   # 连到服务器的 action 名
        self.declare_parameter('id', 1)
        self.declare_parameter('pwm', 1500)
        self.declare_parameter('duration_ms', 0)     # 0 = 用服务端默认值

    def send(self):
        goal = MoveServo.Goal()                      # 造一个 goal 对象
        goal.command = ServoCommand(                 # 填 command 字段（嵌套的 msg）
            id=self.get_parameter('id').value,
            pwm=self.get_parameter('pwm').value,
            duration_ms=self.get_parameter('duration_ms').value,
        )
        if not self._client.wait_for_server(timeout_sec=3.0):   # 等服务端在线
            self.get_logger().error('Action Server 不在线')
            return
        # 异步发 goal，收到反馈就调 self._fb
        self._client.send_goal_async(goal, feedback_callback=self._fb).add_done_callback(self._goal_done)

    def _fb(self, fb_msg):
        fb = fb_msg.feedback            # 注意：要取 .feedback 才是真正的反馈字段
        self.get_logger().info(f'feedback: id={fb.id} pwm={fb.current_pwm} progress={fb.progress:.2f}')

    def _goal_done(self, future):
        goal_handle = future.result()   # 这里拿到的是"goal 有没有被接受"
        if not goal_handle.accepted:
            self.get_logger().error('goal 被拒')
            return
        goal_handle.get_result_async().add_done_callback(self._result_done)   # 再异步等 result

    def _result_done(self, future):
        result = future.result().result  # 这里拿到的才是最终 result
        self.get_logger().info(f'result: success={result.success} msg={result.message} final_pwm={result.final_pwm}')
def main(args=None):
    rclpy.init(args=args)                 # 初始化 ROS2（必须最先做）
    node = MoveClient()                   # 创建客户端节点
   
    try:
        node.send()                           # 主动发一个 goal（客户端和服务器唯一不同的地方）
        rclpy.spin(node)                  # 转起来，等 feedback / result 回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
if __name__ == '__main__':
    main()
