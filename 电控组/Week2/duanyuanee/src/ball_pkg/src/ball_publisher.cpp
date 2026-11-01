#include "rclcpp/rclcpp.hpp"
#include "ball_pkg/msg/ball_pos.hpp"

class BallPublisher : public rclcpp::Node
{
public:
  BallPublisher()
  : Node("ball_publisher_node"), current_index_(0)
  {
    // 1. 创建发布者
    publisher_ = this->create_publisher<ball_pkg::msg::BallPos>("ball_pos", 10);
    
    // 2. 创建定时器，每 1 秒发布一次数据
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&BallPublisher::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "3个球的坐标发布器已启动！");
  }

private:
  void timer_callback()
  {
    // 1. 创建消息
    auto message = ball_pkg::msg::BallPos();
    
    // 2. 按照表格要求的坐标
    int temp_ids[] = {1,2,3};
    float temp_xs[] = {2.0,5.5,9.0};  // 分别对应1号、2号、3号球的x坐标
    float temp_ys[] = {3.0,8.0,4.0};  // 分别对应1号、2号、3号球的y坐标
    
    // 通过current_index_ 依次取出第几个球的数据
    message.id = temp_ids[current_index_];
    message.x = temp_xs[current_index_];
    message.y = temp_ys[current_index_];
  
    // 3. 发布消息并打印日志
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "发布: id=%d,x=%.2f, y=%.2f", message.id, message.x, message.y);

    // 4. 更新索引，实现 0->1->2->0 的循环
    current_index_ = (current_index_ + 1) % 3;
  }

  rclcpp::Publisher<ball_pkg::msg::BallPos>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int current_index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallPublisher>());
  rclcpp::shutdown();
  return 0;
}