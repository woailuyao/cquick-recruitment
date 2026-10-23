import time, rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('my_talker')
pub_cmd = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
pub_log = node.create_publisher(String, '/my_chatter', 10)

msg = Twist()
msg.linear.x = 1.0
msg.angular.z = 0.8
t0 = time.time()
n = 0
while time.time() - t0 < 6.0:
    s = String()
    s.data = 'hello ros2 #%d' % n
    pub_log.publish(s)
    pub_cmd.publish(msg)
    print('[发布] /turtle1/cmd_vel  linear.x=%.1f angular.z=%.1f | /my_chatter: %s' % (msg.linear.x, msg.angular.z, s.data))
    n += 1
    time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
print('[发布] 结束，共发布 %d 条' % n)
