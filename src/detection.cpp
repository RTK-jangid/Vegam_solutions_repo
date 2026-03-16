#include "vegam_solution_assignment/detection.hpp"

Detection::Detection(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    declareParameters();

    // Initialize the bot

    bot_ = std::make_shared<ContainerBot>(node_);
    RCLCPP_INFO(node_->get_logger(), "Bot Started!");

    // Initialize the subscriber to /scan
    scan_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&Detection::laserScanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Detection Node Initialized: Subscribed to /scan");
    startMainLoop();
}

void Detection::declareParameters()
{
    node_->declare_parameter("speed", 0.22);
    node_->declare_parameter("angular_velocity", 1.0);
    node_->declare_parameter("check_points", 50);
}

void Detection::startMainLoop()
{   
    while(!entrance_detected){
        RCLCPP_INFO(node_->get_logger(), "Debug Point %d", middle_point);
    }
    
    enterContainer();
    exitContainer();
    

}

void Detection::startScanning()
{
}

void Detection::alignWithContainer()
{
    
    while (abs(450 - middle_point) > 10)
    {
        if (450 - middle_point > 0)
        {
            bot_->rotateLeft(1.0);
        }
        else
        {
            bot_->rotateRight(1.0);
        }
    }
    bot_->stop();
}

void Detection::enterContainer()
{
    alignWithContainer();
    while (scan_->ranges[450] > 2.0)
    {
        bot_->moveForward(0.5);
    }
    bot_->stop();
}

void Detection::exitContainer()
{
    alignWithContainer();
    while (scan_->ranges[450] > 2.0)
    {
        bot_->moveForward(0.5);
    }
    bot_->stop();
}

int Detection::detectContainerEntrance(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_)
{

    double jump_threshold = 1.0; // Meters. Adjust based on your container depth.
    std::vector<int> corner_indices;

    for (size_t i = 1; i < laser_scan_->ranges.size(); ++i)
    {
        // Skip invalid readings (NaN or Inf)
        if (std::isinf(laser_scan_->ranges[i]) || std::isinf(laser_scan_->ranges[i - 1]))
            continue;

        double diff = std::abs(laser_scan_->ranges[i] - laser_scan_->ranges[i - 1]);

        if (diff > jump_threshold)
        {
            if ((laser_scan_->ranges[i] - laser_scan_->ranges[i - 1]) > 0)
            {
                corner_indices.push_back(i - 1);
                RCLCPP_INFO(node_->get_logger(), "Potential corner at index and distance: %ld , %.2f", i - 1, laser_scan_->ranges[i - 1]);
            }
            else
            {
                corner_indices.push_back(i);
                RCLCPP_INFO(node_->get_logger(), "Potential corner at index and distance: %ld , %.2f", i, laser_scan_->ranges[i]);
            }
        }
    }

    bool detected_in_the_loop = false;
    int middle_point;

    for (size_t i = 1; i < corner_indices.size(); i++)
    {
        auto cornor1 = corner_indices[i - 1];
        auto cornor2 = corner_indices[i];

        if (abs(cornor1 - cornor2) < 400)
        {
            middle_point = (cornor1 + cornor2) / 2;
        }
        else
        {

            middle_point = (cornor2 + (cornor1 + (900 - cornor2)) / 2) % 900;
        }

        if ((laser_scan_->ranges[middle_point] > laser_scan_->ranges[cornor1]) && (laser_scan_->ranges[middle_point] > laser_scan_->ranges[cornor2]))
        {
            if (!entrance_detected)
            {
                if (confidence >= 20)
                {
                    entrance_detected = true;
                }
                confidence++;
            }

            detected_in_the_loop = true;
            RCLCPP_INFO(node_->get_logger(), "middle point at index and distance: %d , %.2f", middle_point, laser_scan_->ranges[middle_point]);
            RCLCPP_INFO(node_->get_logger(), "ENTRANCE DETECTED! Confidence: %d /20", confidence);
        }
    }
    if (!detected_in_the_loop && confidence > 0)
    {
        reset_count++;
        if (reset_count > 3)
        {
            confidence = 0;
            reset_count = 0;
        }
    }
    return middle_point;
}

void Detection::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // This function triggers every time a new scan is published
    middle_point = detectContainerEntrance(msg);
    scan_ = msg;
}
