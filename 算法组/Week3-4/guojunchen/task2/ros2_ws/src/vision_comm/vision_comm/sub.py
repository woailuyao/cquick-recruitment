import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition, FieldPose


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.create_subscription(TargetPosition, '/target_position', self.target_callback, 10)
        self.create_subscription(FieldPose, '/field_pose', self.pose_callback, 10)

    def target_callback(self, msg):
        self.get_logger().info(f'接收: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

    def pose_callback(self, msg):
        self.get_logger().info(
            f'位姿: valid={msg.valid}, x={msg.x:.2f}, y={msg.y:.2f}, '
            f'yaw={msg.yaw:.1f}, markers={msg.marker_count}, status={msg.status}')


def main():
    rclpy.init()
    rclpy.spin(TargetSubscriber())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
