#include "rclcpp/rclcpp.hpp"
#include "ball_pkg/msg/ball_pos.hpp"

class BallSubscriber : public rclcpp::Node
{
public:
  BallSubscriber()
  : Node("ball_subscriber_node")
  {
    // 1. 创建订阅者，监听 "ball_position" 话题
    subscription_ = this->create_subscription<ball_pkg::msg::BallPos>(
      "ball_pos", 10,
      std::bind(&BallSubscriber::topic_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "球坐标订阅器已启动，等待接收数据...");
  }

private:
  void topic_callback(const ball_pkg::msg::BallPos::SharedPtr msg) const
  {
    // 2. 收到消息后，打印出球的 ID 和坐标
    RCLCPP_INFO(this->get_logger(), "收到球[ID=%d]的位置: [x=%.2f, y=%.2f]", msg->id, msg->x, msg->y);
  }

  rclcpp::Subscription<ball_pkg::msg::BallPos>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallSubscriber>());
  rclcpp::shutdown();
  return 0;
}