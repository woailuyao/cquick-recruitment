import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._client.wait_for_server()   # 等服务端就绪（阻塞等，简单够用）
        self._sid = 1                    # feedback 里没有 id，自己记一份用来打印

    def send_goal(self, sid, pwm, duration_ms):
        goal_msg = MoveServo.Goal()
        goal_msg.command.id = sid
        goal_msg.command.pwm = pwm
        goal_msg.command.duration_ms = duration_ms
        self._sid = sid

        self.get_logger().info(
            '发送 goal: id=%d pwm=%d dur=%d' % (sid, pwm, duration_ms))

        # 异步发送，绑定 feedback 回调；拿到句柄后再去要 result
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 被拒绝！')
            return
        self.get_logger().info('goal 被接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            'feedback: id=%d pwm=%d progress=%.2f'
            % (self._sid, fb.current_pwm, fb.progress))

    def result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('结果：成功，实际 pwm=%d' % result.actual_pwm)
        else:
            self.get_logger().warn('结果：失败，原因=%s' % result.message)


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    # 从命令行读 id pwm duration（demo 用途）
    # launch 用 name='move_client' 起它时会塞 --ros-args -r __node:=move_client，
    # 直接 int(sys.argv[1]) 会撞上 '--ros-args' 报 ValueError，所以先过滤 ROS 参数
    plain = [a for a in sys.argv[1:] if not a.startswith('-') and ':=' not in a]
    sid = int(plain[0]) if len(plain) > 0 else 1
    pwm = int(plain[1]) if len(plain) > 1 else 1700
    dur = int(plain[2]) if len(plain) > 2 else 800
    node.send_goal(sid, pwm, dur)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
