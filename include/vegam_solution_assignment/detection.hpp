#ifndef DETECTION_HPP
#define DETECTION_HPP

#include <chrono>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vegam_solution_assignment/container_bot.hpp"

using namespace std::chrono_literals;

class Detection
{
public:
    Detection(const rclcpp::Node::SharedPtr &node);
    ~Detection() = default;

private:
    void declareParameters();

    void startMainLoop();

    void startScanning();

    void alignWithContainer();

    void enterContainer();

    void exitContainer();

    int detectContainerEntrance(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_);

    // Callback function for the subscriber
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /// Shared pointer to the ROS 2 node
    rclcpp::Node::SharedPtr node_;

    // Subscriber for the LIDAR data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    // Instance of ContainerBot to call movement functions
    std::shared_ptr<ContainerBot> bot_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    float distance_to_front;

    bool entrance_detected = false;

    int confidence = 0;

    int reset_count = 0;

    int middle_point = 0;
};

#endif
