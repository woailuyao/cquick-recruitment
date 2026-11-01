import rclpy


from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from target_position.msg import TargetPosition

#创建listener类
class ListenerNode(Node):

    def __init__(self):
        super().__init__("target_listener")

        self.sub = self.create_subscription(
            TargetPosition,#消息类型
            "/target_position",#话题
            self.on_msg,#回调
            10,#限制队列长
        )

        self.count = 0#计数，算队长

    def on_msg(self, msg: TargetPosition):

        self.count += 1#count++

        if msg.detected:#收到目标就开始打印字幕
            self.get_logger().info(
                f"#{self.count} 目标中心 "
                f"(u={msg.u:.1f}, v={msg.v:.1f}) 面积={msg.area:.0f}")
        else:#没有目标打印黄色warn信息
            self.get_logger().warn(f"#{self.count} 未检测到目标")

#主函数main
def main():
    rclpy.init()#初始化

    node = ListenerNode()#创建节点

    try:
        rclpy.spin(node)#死循环
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:#无论如何最后都要清理进程，防止大量在线导致MLE
        try:
            node.destroy_node()#释放订阅者和节点资源
            rclpy.shutdown()#关闭 rclpy
        except Exception:
            pass

if __name__ == "__main__":
    main()

