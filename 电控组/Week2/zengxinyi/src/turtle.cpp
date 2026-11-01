#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "zxy/msg/ball_pos.hpp"
using namespace std::chrono_literals;

class Turtle : public rclcpp::Node
{
public:

    Turtle() : Node("turtle")
    {
        ball = create_subscription<zxy::msg::BallPos>
        (
            "ball_pos",10,
            std::bind
            (
                &Turtle::ball_callback,this,std::placeholders::_1
            )
        );
         pose = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",10,
            std::bind
            (
                &Turtle::pose_callback,this,std::placeholders::_1
            )
        );
        
        velocity = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

        timer = create_wall_timer(20ms, [this]() {control_callback();});

    }

private:

    void ball_callback(const zxy::msg::BallPos::SharedPtr msg)
    {
       const std::size_t index = static_cast<std::size_t>(msg->id - 1);
       ball_x_[index] = msg->x;
       ball_y_[index] = msg->y;
       ball_received_[index] = true;
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void control_callback()
    {
        if (finished_){return;}

        if (!pose_received_ || !ball_received_[0] || !ball_received_[1] || !ball_received_[2])
        {return;}

        double target_x = ball_x_[current_target_];
        double target_y = ball_y_[current_target_];
        double dx = target_x - current_pose_.x;
        double dy = target_y - current_pose_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
         
        if (distance < 0.5 )
        {
            auto stop_command = geometry_msgs::msg::Twist();
            velocity ->publish(stop_command);
            RCLCPP_INFO(get_logger(),"Arrive ball%d, error = %.2f",current_target_ + 1,distance);

            current_target_++;

            if (current_target_ == 3)
            {
                finished_ = true;
            }
        }else
        {
            double target_angle = std::atan2(dy, dx);
            double angle = target_angle - current_pose_.theta;
            const double pi = 3.1415926535;

            while (angle > pi){angle -= 2.0 * pi;}
            while (angle < -pi){angle += 2.0 * pi;}

            auto command = geometry_msgs::msg::Twist();
            if (std::abs(angle) > 0.4)
            {
                command.linear.x = 0.0;
            }else
            {
                double speed = 1.5 * distance;
            
                if (speed > 1.5)
                {
                    speed = 1.5;
                }
            
                command.linear.x = speed;
            
            }
        
            command.angular.z = 4.0 * angle;
            velocity ->publish(command);
        }
    }

    double ball_x_[3] = {0.0, 0.0, 0.0};
    double ball_y_[3] = {0.0, 0.0, 0.0};

    bool ball_received_[3] = {false, false, false};
    int current_target_ = 0;

    turtlesim::msg::Pose current_pose_;

    bool pose_received_ = false;
    bool finished_ = false;

    rclcpp::Subscription<zxy::msg::BallPos>::SharedPtr ball ;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose ;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity ;
    rclcpp::TimerBase::SharedPtr timer ;
};


    int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtle>());
    rclcpp::shutdown();
    return 0;
}