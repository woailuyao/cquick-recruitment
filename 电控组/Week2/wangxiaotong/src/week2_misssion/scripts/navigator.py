#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from week2_mission1.msg import BallPos
import math

class TurleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')

        self.sub_ball=self.create_subscription(BallPos,'ball_pos',self.ball_callback,10)
        self.sub_pose=self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
        self.pub_cmd=self.create_publisher(Twist,'/turtle1/cmd_vel',10)

        self.balls={}
        self.target_order=[1,2,3]
        self.current_target_idx=0
        self.cur_x=0.0
        self.cur_y=0.0
        self.cur_theda=0.0

        self.dist_threshold=0.1
        self.max_linear=2.0
        self.max_angular=2.0

    def ball_callback(self,msg):
        self.balls[msg.id]=(msg.x,msg.y)
        #self.get_logger().info(f'导航节点收到球{msg.id}坐标')
        if len(self.balls)==3 and self.current_target_idx == 0:
            self.get_logger().info('收了三个球了，准备启动')

    def pose_callback(self,msg):
        self.cur_x=msg.x
        self.cur_y=msg.y
        self.cur_theta=msg.theta

        if len(self.balls)<3:
            return
        if self.current_target_idx>=len(self.target_order):
            self.stop()
            return
        
        target_id=self.target_order[self.current_target_idx]
        target_x,target_y=self.balls[target_id]

        dx=target_x-self.cur_x
        dy=target_y-self.cur_y
        distance=math.hypot(dx,dy)

        if distance<self.dist_threshold:
            self.get_logger().info(f'到达球{target_id},实际坐标({self.cur_x:.2f},{self.cur_y:.2f}),误差{distance:.2f}')
            self.current_target_idx+=1
            self.stop()
            return

        target_theta=math.atan2(dy,dx)
        angle_diff=target_theta-self.cur_theta

        while angle_diff>math.pi:
            angle_diff-=2.0*math.pi

        while angle_diff<-math.pi:
            angle_diff+=2.0*math.pi


        cmd=Twist()

        if abs(angle_diff)>0.5:
            cmd.linear.x=0.0
            cmd.angular.z=1.5*angle_diff
        else:
            cmd.linear.x=1.0*distance
            cmd.angular.z=2.0*angle_diff

        cmd.linear.x=min(cmd.linear.x,self.max_linear)
        cmd.angular.z=max(min(cmd.angular.z,self.max_angular),-self.max_angular)#限制速度大小同时选择最优方向

        self.pub_cmd.publish(cmd)

    def stop(self):
        cmd=Twist()
        cmd.linear.x=0.0
        cmd.angular.z=0.0
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=TurleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()