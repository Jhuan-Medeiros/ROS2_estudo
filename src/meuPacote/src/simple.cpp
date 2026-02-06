#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Obiwan");
    rclcpp::WallRate loop_rate(2);

    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
    
}