#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class MinimalSubscriber: public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"){
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
      subscription_2 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::QoS(10), std::bind(&MinimalSubscriber::topic_callback_2, this, std::placeholders::_1)); 

      //auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    }
    private:
      void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
          RCLCPP_INFO(this->get_logger(), "Angle_min: %.3f", msg->angle_min);
          if (!msg->ranges.empty()) {
            RCLCPP_INFO(this->get_logger(), "Range[0]: %.3f", msg->ranges[0]);
          } else {
            RCLCPP_WARN(this->get_logger(), "LaserScan ranges[] is empty");
          }
        }

      void topic_callback_2(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->longitude); // %s + c_str()
      }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_2;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
