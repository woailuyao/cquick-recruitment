#include<chrono>
#include<memory>
#include"rclcpp/rclcpp.hpp"
#include"zxy/msg/ball_pos.hpp"
using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("publisher")
    {
        ball_pos = create_publisher<zxy::msg::BallPos>("ball_pos",10);
        timer_ = create_wall_timer(200ms, [this](){timer_callback();});
    }

private:
    void timer_callback()
    { 
        auto ball1 = zxy::msg::BallPos{}; 
        ball1.x = 2.0; 
        ball1.y = 3.0;
        ball1.id = 1;
        ball_pos->publish(ball1);
        RCLCPP_INFO(get_logger(),"id = %d, x=%f, y=%f",ball1.id,ball1.x,ball1.y);

        auto ball2 = zxy::msg::BallPos{}; 
        ball2.x = 5.5; 
        ball2.y = 8.0;
        ball2.id = 2;
        ball_pos->publish(ball2);
        RCLCPP_INFO(get_logger(),"id = %d, x=%f, y=%f",ball2.id,ball2.x,ball2.y );

        auto ball3 = zxy::msg::BallPos{}; 
        ball3.x = 9.0; 
        ball3.y = 4.0;
        ball3.id = 3;
        ball_pos->publish(ball3);
        RCLCPP_INFO(get_logger(), "id = %d, x=%f, y=%f",ball3.id,ball3.x,ball3.y);

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<zxy::msg::BallPos>::SharedPtr ball_pos;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}