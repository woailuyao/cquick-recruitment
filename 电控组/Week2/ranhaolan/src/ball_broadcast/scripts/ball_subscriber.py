#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from ball_broadcast.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.subscription = self.create_subscription(BallPos, 'ball_pos', self.callback, 10)
    def callback(self,msg):
        self.get_logger().info('Received ball %s:x=%s,y=%s'%(msg.id,msg.x,msg.y))
def main(args=None):
    rclpy.init(args=args)
    node=BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
if __name__=='__main__':
    main()
