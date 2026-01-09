#include "lds_lidar/lds_lidar.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <algorithm>

// Neato XV-11 / LDS definitions
static const uint8_t START_BYTE = 0xFA;
static const size_t PACKET_SIZE = 22;

LDS_Lidar::LDS_Lidar() 
    : Node("lds_lidar") 
{
    // Declare Parameters
    this->declare_parameter<std::string>("port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::string>("gpio_chip", "gpiochip0");
    this->declare_parameter<int>("gpio_line", 13); // -1 to disable GPIO usage

    std::string port = this->get_parameter("port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    std::string gpio_chip_name = this->get_parameter("gpio_chip").as_string();
    int gpio_line_num = this->get_parameter("gpio_line").as_int();

    // Initialize GPIO (Motor Enable) if requested
    if (gpio_line_num >= 0) {
        chip = gpiod_chip_open_by_name(gpio_chip_name.c_str());
        if (chip) {
            line = gpiod_chip_get_line(chip, gpio_line_num);
            if (line) {
                // Request output mode and set HIGH (1) to enable motor
                if (gpiod_line_request_output(line, "lds_lidar", 1) == 0) {
                    RCLCPP_INFO(this->get_logger(), "GPIO Motor enabled on %s line %d", gpio_chip_name.c_str(), gpio_line_num);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to request GPIO line");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get GPIO line");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip");
        }
    }

    // --- Publish static TF for laser_link ---
    /*tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";       // parent
    t.child_frame_id = "laser_link"; // child
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);*/

    // Initialize UART
    if (!init_uart(port, baud)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open UART %s", port.c_str());
        // In a real node, you might throw or exit, but we keep it running to allow diagnostics
    } else {
        RCLCPP_INFO(this->get_logger(), "UART initialized on %s at %d", port.c_str(), baud);
    }

    // Setup ROS Interfaces
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    
    // Timer (20ms) to poll serial buffer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&LDS_Lidar::timer_callback, this));
}

LDS_Lidar::~LDS_Lidar() {
    if (uart_fd >= 0) {
        close(uart_fd);
    }

    // Cleanup GPIO
    if (line) {
        gpiod_line_set_value(line, 0); 
        gpiod_line_release(line);
    }
    if (chip) {
        gpiod_chip_close(chip);
    }
}

bool LDS_Lidar::init_uart(const std::string &port, int baud) {
    uart_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(uart_fd, &tty) != 0) return false;

    // Handle Baud Rate
    speed_t speed;
    switch(baud) {
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  
    tty.c_iflag &= ~IGNBRK;                      
    tty.c_lflag = 0;                             
    tty.c_oflag = 0;                             
    tty.c_cc[VMIN]  = 0;                         
    tty.c_cc[VTIME] = 0;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      
    tty.c_cflag |= (CLOCAL | CREAD);             
    tty.c_cflag &= ~(PARENB | PARODD);           
    tty.c_cflag &= ~CSTOPB;                      
    tty.c_cflag &= ~CRTSCTS;                     

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) return false;
    
    return true;
}

uint16_t LDS_Lidar::calcChecksum(const uint8_t* p) {
    // 20 bytes data, last 2 bytes checksum
    const int CalcCRC_Len = 10;
    uint16_t CalcCRC[CalcCRC_Len];

    for (int i = 0; i < CalcCRC_Len; ++i)
        CalcCRC[i] = p[2 * i] + (p[2 * i + 1] << 8);

    uint32_t chk32 = 0;
    for (int i = 0; i < CalcCRC_Len; ++i)
        chk32 = (chk32 << 1) + CalcCRC[i];

    uint16_t checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum &= 0x7FFF;
    return checksum;
}

bool LDS_Lidar::parsePacket(const uint8_t* p, size_t len, std::vector<LidarPoint> &out) {
    if (len < PACKET_SIZE) return false;
    if (p[0] != START_BYTE) return false;
    if (p[1] < 0xA0 || p[1] > 0xF9) return false;

    uint16_t actual = calcChecksum(p);
    uint16_t received = p[20] | (p[21] << 8);

    if (actual != received) return false;

    uint8_t index = p[1];
    double base_angle = (index - 0xA0) * 4.0; 

    // 4 data points per packet
    for (int i = 0; i < 4; ++i) {
        int offset = 4 + i * 4;
        
        // Skip points with "Invalid Data" flag (bit 15)
        if (p[offset+1] & 0x80) continue; 

        uint16_t dist = p[offset] | ((p[offset + 1] & 0x3F) << 8);
        uint8_t quality = p[offset + 2];

        // Filtering
        if (quality < 10 || dist == 0 || dist > 6000) continue;

        float angle = base_angle + i;
        if (angle >= 360.0) angle -= 360.0;

        out.push_back({angle, dist, quality});
    }

    return true;
}

void LDS_Lidar::readLidarData(std::vector<LidarPoint> &points) {
    if (uart_fd < 0) return;

    // Static buffer to hold data between timer calls
    static std::vector<uint8_t> rxBuffer;
    uint8_t tmp_buf[256];

    // Read available data
    int n = ::read(uart_fd, tmp_buf, sizeof(tmp_buf));
    if (n > 0) {
        rxBuffer.insert(rxBuffer.end(), tmp_buf, tmp_buf + n);
    }

    // Process Buffer
    while (rxBuffer.size() >= PACKET_SIZE) {
        auto it = std::find(rxBuffer.begin(), rxBuffer.end(), START_BYTE);
        
        // If no header found, clear garbage
        if (it == rxBuffer.end()) {
            rxBuffer.clear();
            break;
        }

        size_t idx = std::distance(rxBuffer.begin(), it);
        
        // Wait for full packet
        if (rxBuffer.size() - idx < PACKET_SIZE) {
            if (idx > 0) rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + idx);
            break; 
        }

        // Try to parse
        if (parsePacket(&rxBuffer[idx], PACKET_SIZE, points)) {
            // Success, remove packet
            rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + idx + PACKET_SIZE);
        } else {
            // Checksum failed or invalid, move forward 1 byte
            rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + idx + 1);
        }
    }
}

void LDS_Lidar::timer_callback() {
    std::vector<LidarPoint> new_points;
    readLidarData(new_points);

    // Static structures to persist scan construction across timer calls
    static sensor_msgs::msg::LaserScan scan_msg;
    static bool initialized = false;
    static float prev_angle = 0.0f;

    if (!initialized) {
        scan_msg.header.frame_id = "laser_link";
        scan_msg.angle_min = 0.0;
        scan_msg.angle_max = 2.0 * M_PI;
        scan_msg.angle_increment = (2.0 * M_PI / 360.0);
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 6.0;
        scan_msg.ranges.assign(360, std::numeric_limits<float>::infinity());
        scan_msg.intensities.assign(360, 0.0);
        initialized = true;
    }

    for (const auto& p : new_points) {
        // Check for wrap-around (start of new scan)
        if (p.angle < prev_angle - 10.0f) { // Simple heuristic for wrapping
            scan_msg.header.stamp = this->now();
            publisher_->publish(scan_msg);

            // Reset for next scan
            std::fill(scan_msg.ranges.begin(), scan_msg.ranges.end(), std::numeric_limits<float>::infinity());
            std::fill(scan_msg.intensities.begin(), scan_msg.intensities.end(), 0.0);
        }

        int index = static_cast<int>(p.angle + 0.5f) % 360;
        scan_msg.ranges[index] = p.dist / 1000.0f; // mm to meters
        scan_msg.intensities[index] = static_cast<float>(p.quality);
        
        prev_angle = p.angle;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LDS_Lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}