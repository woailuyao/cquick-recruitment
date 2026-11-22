#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from ball_broadcast.msg import BallPos

BALLS=[(1,2.0,3.0),(2,5.5,8.0),(3,9.0,4.0)]
class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('period', 0.5)
        self.publisher = self.create_publisher(BallPos, 'ball_pos', 10)
        period = self.get_parameter('period').value
        self.timer = self.create_timer(period, self.timer_callback)


    def timer_callback(self):
        for id,x,y in BALLS:
            msg=BallPos()
            msg.id=id
            msg.x=x
            msg.y=y
            self.publisher.publish(msg)
            self.get_logger().info('Published ball %s:x=%s,y=%s'%(id,x,y))
def main(args=None):
    rclpy.init(args=args)
    node=BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__=='__main__':
     main()
