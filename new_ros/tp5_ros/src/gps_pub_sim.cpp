#include <iostream>
#include <thread>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h> // For ioctl
#include <unistd.h>    // For read/write/close
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

double globalLongitude = 17.0171259;
double globalLatitude = 48.1540940;
double globalHorizontalAccuracy = 0.0;

// Global variables
int globalGroundSpeed = 0;
uint32_t globalCourse = 0;

class GpsPublisher : public rclcpp::Node
{
public:
    GpsPublisher() : Node("gps_pub")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&GpsPublisher::timer_callback, this));
        std::cout << "Publisher start" << std::endl;
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::NavSatFix();

        msg.header.stamp = this->now();

        msg.latitude = globalLatitude;
        msg.longitude = globalLongitude;
        msg.altitude = globalHorizontalAccuracy;
        msg.position_covariance_type = globalCourse;


        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gps_publisher = std::make_shared<GpsPublisher>();
    rclcpp::spin(gps_publisher);
    rclcpp::shutdown();
    return 0;
}

