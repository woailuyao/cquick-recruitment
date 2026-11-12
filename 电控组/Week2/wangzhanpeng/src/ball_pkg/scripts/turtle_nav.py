#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        self.cmd_pub=self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.ball_sub=self.create_subscription(BallPos,'ball_pos',self.ball_callback,10)
        self.pose_sub=self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)

        self.balls={}
        self.current_pose=None
        self.target_id=1
        self.reached=False

        self.timer=self.create_timer(0.05,self.control)
    
    def ball_callback(self,msg):
        self.balls[msg.id]=(msg.x,msg.y)
    def pose_callback(self,msg):
        self.current_pose=msg

    def control(self):
        if len(self.balls)<3 or self.current_pose is None:
            return
        if self.target_id>3:
            self.stop()
            return
        target_x,target_y=self.balls[self.target_id]
        cur_x=self.current_pose.x
        cur_y=self.current_pose.y
        cur_theta=self.current_pose.theta
        d=math.sqrt((target_x-cur_x)**2+(target_y-cur_y)**2)
        target_theta=math.atan2(target_y-cur_y,target_x-cur_x)
        diff=target_theta-cur_theta

        
        cmd=Twist()

        if d<0.3:
            if self.reached==False:
                self.get_logger().info(f'reach ball{self.target_id}! pose:({cur_x:.2f},{cur_y:.2f}),dist:{d:.2f}')
                self.reached=True
                self.target_id+=1
                self.stop()
            return
        if abs(diff)>0.2:
            cmd.linear.x=0.0
            cmd.angular.z=1.0*diff
            self.reached=False
        else:
            cmd.linear.x=1.0*d
            if cmd.linear.x>2.0:
                cmd.linear.x=2.0
            cmd.angular.z=1.0*diff

        self.cmd_pub.publish(cmd)
    def stop(self):
        cmd=Twist()
        cmd.linear.x=0.0
        cmd.angular.z=0.0
        self.cmd_pub.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node=TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
            main()
