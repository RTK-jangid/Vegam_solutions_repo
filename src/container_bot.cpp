#include "vegam_solution_assignment/container_bot.hpp"

ContainerBot::ContainerBot(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    // Initialize the publisher on the /cmd_vel topic (standard for diff drive)
    velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a timer to call publishCommand at 10Hz (0.1s interval)
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ContainerBot::publishCommand, this));

    RCLCPP_INFO(node_->get_logger(), "ContainerBot Controller Initialized.");
}

void ContainerBot::moveForward(float velocity)
{
    current_twist_.linear.x = velocity; // velocity m/s
    current_twist_.angular.z = 0.0;
    publishCommand();
}

void ContainerBot::moveBackward(float velocity)
{
    current_twist_.linear.x = -velocity; // -velocity m/s
    current_twist_.angular.z = 0.0;
    publishCommand();
}

void ContainerBot::rotateRight(float angular_velocity)
{
    current_twist_.linear.x = 0.0;
    current_twist_.angular.z = -angular_velocity; // angular_velocity rad/s
    publishCommand();
}

void ContainerBot::rotateLeft(float angular_velocity)
{
    current_twist_.linear.x = 0.0;
    current_twist_.angular.z = angular_velocity; // -angular_velocity rad/s
    publishCommand();
}

void ContainerBot::stop()
{
    current_twist_.linear.x = 0.0;
    current_twist_.angular.z = 0.0;
    publishCommand();
}

void ContainerBot::publishCommand()
{
    // Send the current command to the robot
    velocity_publisher_->publish(current_twist_);
}
