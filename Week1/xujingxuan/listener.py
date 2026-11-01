import time, rclpy
from turtlesim.msg import Pose
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('my_listener')

def cb_pose(m):
    print('[订阅] /turtle1/pose  x=%.2f y=%.2f theta=%.2f' % (m.x, m.y, m.theta))

def cb_chat(m):
    print('[订阅] /my_chatter  收到: %s' % m.data)

node.create_subscription(Pose, '/turtle1/pose', cb_pose, 10)
node.create_subscription(String, '/my_chatter', cb_chat, 10)

t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
