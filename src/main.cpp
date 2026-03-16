#include "vegam_solution_assignment/detection.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("container_bot");
    RCLCPP_INFO(node->get_logger(), "Node_Started");
    
    // This single object now handles sensing AND movement
    auto system = std::make_shared<Detection>(node);

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}