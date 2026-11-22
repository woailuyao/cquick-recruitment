#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ray_ball.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Navigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.balls = {}
        self.pose = None
        self.order = [1, 2, 3]
        self.idx = 0
        self.done = False
        self.errs = []
        self.t0 = None
        self.lin_max = 3.0
        self.ang_max = 2.0
        self.dist_th = 0.15
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.05, self.control)

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_cb(self, msg):
        self.pose = msg

    def summary(self):
        log = self.get_logger()
        log.info('===== ERROR SUMMARY =====')
        total = 0.0
        for tid, tx, ty, px, py, d in self.errs:
            total += d
            flag = 'OK' if d <= 0.5 else 'FAIL'
            log.info('球 %d target(%.2f, %.2f) actual(%.2f, %.2f) err=%.3f %s'
                     % (tid, tx, ty, px, py, d, flag))
        if self.errs:
            log.info('平均误差=%.3f  最大误差=%.3f  误差限制=0.500'
                     % (total / len(self.errs), max(e[5] for e in self.errs)))
        if self.t0 is not None:
            sec = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            log.info('总用时=%.1f s' % sec)
        log.info('=========================')

    def control(self):
        if self.pose is None or len(self.balls) < 3:
            return
        if self.idx >= len(self.order):
            self.pub_vel.publish(Twist())
            if not self.done:
                self.done = True
                self.summary()
            return
        tid = self.order[self.idx]
        gx, gy = self.balls[tid]
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy)
        ang_err = math.atan2(dy, dx) - self.pose.theta
        while ang_err > math.pi:
            ang_err -= 2.0 * math.pi
        while ang_err < -math.pi:
            ang_err += 2.0 * math.pi

        if dist < self.dist_th:
            self.get_logger().info('到达球 %d actual(%.2f, %.2f) 误差=%.3f'
                                   % (tid, self.pose.x, self.pose.y, dist))
            self.errs.append((tid, gx, gy, self.pose.x, self.pose.y, dist))
            self.idx += 1
            self.pub_vel.publish(Twist())
            return

        if self.t0 is None:
            self.t0 = self.get_clock().now()

        tw = Twist()
        if abs(ang_err) > 0.15:
            tw.angular.z = self.ang_max if ang_err > 0 else -self.ang_max
            tw.linear.x = 0.0
        else:
            tw.angular.z = 1.5 * ang_err
            tw.linear.x = min(self.lin_max, 0.8 * dist)
        self.pub_vel.publish(tw)


def main():
    rclpy.init()
    rclpy.spin(Navigator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
