#ifndef CONTAINER_BOT_HPP
#define CONTAINER_BOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ContainerBot
{
public:
    ContainerBot(const rclcpp::Node::SharedPtr &node);

    ~ContainerBot() = default;

    void moveForward(float velocity);

    void moveBackward(float velocity);

    void rotateRight(float angular_velocity);

    void rotateLeft(float angular_velocity);

    void stop();

private:
    void publishCommand();

    /// Shared pointer to the ROS 2 node
    rclcpp::Node::SharedPtr node_;
    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    // Timer to run the control loop
    rclcpp::TimerBase::SharedPtr timer_;
    // Current message to be sent
    geometry_msgs::msg::Twist current_twist_;
};

#endif
