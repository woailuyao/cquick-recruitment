#define _USE_MATH_DEFINES
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "ball_pkg/msg/ball_pos.hpp"

#include <map>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class TurtleNavigator : public rclcpp::Node
{
public:
  TurtleNavigator()
  : Node("turtle_navigator")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10);

    ball_subscription_ = this->create_subscription<ball_pkg::msg::BallPos>(
      "ball_pos", 10,
      std::bind(&TurtleNavigator::ball_callback, this, _1));

    pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtleNavigator::pose_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TurtleNavigator::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "乌龟领航员启动！等待接收3个球的坐标...");
  }

private:
  void ball_callback(const ball_pkg::msg::BallPos::SharedPtr msg)
  {
    // 防呆：收齐3个且未重复才存入
    if (balls_.size() < 3 && balls_.find(msg->id) == balls_.end()) {
      balls_[msg->id] = *msg;
      RCLCPP_INFO(this->get_logger(),
        "收到球 %d，坐标 (%.2f, %.2f)", msg->id, msg->x, msg->y);
    }
  }

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
    pose_received_ = true;
  }

  void control_loop()
  {
    if (balls_.size() < 3 || !pose_received_) return;

    if (!started_) {
      RCLCPP_INFO(this->get_logger(), "3个球已收齐，开始导航！");
      started_ = true;
    }

    // 3个点走完(1->2->3)，停止
    if (current_target_id_ > 3) {
      stop_turtle();
      return;
    }

    auto it = balls_.find(current_target_id_);
    if (it == balls_.end()) return;
    
    float goal_x = it->second.x;
    float goal_y = it->second.y;

    float dx = goal_x - current_pose_.x;
    float dy = goal_y - current_pose_.y;
    float distance = std::sqrt(dx*dx + dy*dy);

    float target_theta = std::atan2(dy, dx);
    float angle_error = target_theta - current_pose_.theta;
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;

    auto twist = geometry_msgs::msg::Twist();

    // 1. 未对准：原地转
    if (std::abs(angle_error) > 0.1) {
      twist.linear.x = 0.0;
      twist.angular.z = std::clamp(1.5 * angle_error, -2.0, 2.0);
    }
    // 2. 对准但未到：边走边微调 (阈值0.2，满足≤0.5评审)
    else if (distance > 0.2) {
      twist.linear.x = std::min(1.5 * distance, 2.0);
      twist.angular.z = std::clamp(1.5 * angle_error, -2.0, 2.0);
    }
    // 3. 到达：停稳，切下一点
    else {
      stop_turtle();
      RCLCPP_INFO(this->get_logger(),
        "到达球 %d！实际坐标 (%.2f, %.2f)，误差 %.3f",
        current_target_id_, current_pose_.x, current_pose_.y, distance);
      current_target_id_++;
    }

    publisher_->publish(twist);
  }

  void stop_turtle()
  {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<ball_pkg::msg::BallPos>::SharedPtr ball_subscription_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<int, ball_pkg::msg::BallPos> balls_;
  turtlesim::msg::Pose current_pose_;
  bool pose_received_ = false;
  bool started_ = false;
  int current_target_id_ = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleNavigator>());
  rclcpp::shutdown();
  return 0;
}