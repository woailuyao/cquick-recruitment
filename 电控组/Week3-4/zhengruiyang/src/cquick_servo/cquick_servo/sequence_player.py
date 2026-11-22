import os
import yaml                          # 读 yaml 用
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory   # 查包被装到哪了

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.declare_parameter('action_file', 'wave.yaml')   # 默认播哪个动作
        self._steps = []            # 动作步骤列表
        self._step_index = 0        # 当前播到第几步
        self._pending_in_step = 0   # 当前这一步还有几个 goal 没回 result

    def play(self):
        action_file = self.get_parameter('action_file').value
        pkg_share = get_package_share_directory('cquick_servo')   # 查包路径（不用写相对路径）
        path = os.path.join(pkg_share, 'config', 'actions', action_file)
        with open(path, 'r') as f:
            self._steps = yaml.safe_load(f)['steps']   # 读 yaml → Python 列表
        self._step_index = 0
        # 防呆：确认 Action Server 在线再发指令（否则 goal 发出去石沉大海、静默卡死）
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('找不到 Action Server，请先启动 servo_driver')
            return
        self._play_step()

    def _play_step(self):
        if self._step_index >= len(self._steps):       # 播完了
            self.get_logger().info('动作组播放完毕')
            return
        step = self._steps[self._step_index]
        joints = step['joints']
        self._pending_in_step = len(joints)            # 记下本步要等几个 result
        for joint in joints:
            goal = MoveServo.Goal()
            goal.command = ServoCommand(
                id=joint['id'], pwm=joint['pwm'],
                duration_ms=joint.get('duration_ms', 800),
            )
            # 一步之内多个 goal 同时发（并行协同）
            self._client.send_goal_async(goal).add_done_callback(self._goal_done)
        self._step_index += 1

    def _goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 被拒')
            self._one_done()
            return
        goal_handle.get_result_async().add_done_callback(self._result_done)

    def _result_done(self, future):
        result = future.result().result
        now = self.get_clock().now().to_msg()          # 打时间戳，证明"收到 result 才发下一个"
        self.get_logger().info(f'result: success={result.success} msg={result.message} @ {now.sec}.{now.nanosec:09d}')
        self._one_done()

    def _one_done(self):
        self._pending_in_step -= 1                     # 本步完成一个
        if self._pending_in_step <= 0:                 # 本步所有 goal 的 result 都回来了
            self._play_step()                          # 才进下一步（result 当同步信号）
def main(args=None):
    rclpy.init(args=args)                 # 初始化 ROS2（必须最先做）
    node = SequencePlayer()               # 创建播放器节点
    
    try:
        node.play()                           # 主动开始播动作组（播放器和服务器唯一不同的地方）
        rclpy.spin(node)                  # 转起来，等 result 回调
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
