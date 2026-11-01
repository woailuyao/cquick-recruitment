#include <memory>
#include<functional>
#include "rclcpp/rclcpp.hpp"
#include "zxy/msg/ball_pos.hpp" 

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber() : Node("subscriber")
    {  
        subscription = create_subscription<zxy::msg::BallPos>(
            "ball_pos", 10, std::bind(&MySubscriber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const zxy::msg::BallPos::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "id=%d, x=%f, y=%f", msg->id, msg->x, msg->y);
    }
    
    rclcpp::Subscription<zxy::msg::BallPos>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}