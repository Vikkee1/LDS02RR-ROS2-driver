#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cstdint>
#include <string>
#include <gpiod.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

struct LidarPoint {
    float angle;      // degrees
    uint16_t dist;    // mm
    uint8_t quality;
};

class LDS_Lidar : public rclcpp::Node
{
public:
    LDS_Lidar();
    ~LDS_Lidar();

private:
    void timer_callback();
    void readLidarData(std::vector<LidarPoint> &points);
    bool init_uart(const std::string &port, int baud);

    // Corrected signatures from original snippet
    bool parsePacket(const uint8_t* p, size_t len, std::vector<LidarPoint> &out);
    uint16_t calcChecksum(const uint8_t* p);

    int uart_fd{-1};

    // GPIO (C-style libgpiod structs)
    struct gpiod_chip *chip{nullptr};
    struct gpiod_line *line{nullptr};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};
