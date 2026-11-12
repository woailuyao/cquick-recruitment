# 这是一个最基础的函数：只要收到消息，就会执行这里面的代码
def listener_callback(msg):
    # 把接收到的数据，按部就班地打印出来
    print(f'收到球: ID={msg.id}, X={msg.x}, Y={msg.y}')

def main(args=None):
    # 1. 初始化 ROS 2
    rclpy.init(args=args)
    
    # 2. 直接创建一个节点，不需要写复杂的 __init__ 类
    node = rclpy.create_node('ball_subscriber')
    
    # 3. 创建订阅者，告诉它去监听 'ball_pos' 话题
    subscription = node.create_subscription(
        BallPos,
        'ball_pos',
        listener_callback,
        10
    )
    
    # 打印一句启动提示
    node.get_logger().info("订阅者节点已启动，正在等待接收球坐标...")
    
    # 4. 让节点保持运行，一直等待消息
    rclpy.spin(node)
    
    # 5. 退出时清理资源
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()