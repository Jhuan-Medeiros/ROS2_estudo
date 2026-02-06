#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&SimplePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;
        message.angular.z = 0;
        count_++;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}