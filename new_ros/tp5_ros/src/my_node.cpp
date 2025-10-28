#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class MinimalSubscriber: public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"){
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
      subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom", rclcpp::QoS(10), std::bind(&MinimalSubscriber::topic_callback_2, this, std::placeholders::_1)); 

      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      subscription_3 = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&MinimalSubscriber::topic_callback_3, this, std::placeholders::_1));
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

      void topic_callback_2(const nav_msgs::msg::Odometry::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Child_frame_id: %s", msg->child_frame_id.c_str()); // %s + c_str()
        const auto &p = msg->pose.pose.position;
        RCLCPP_INFO(this->get_logger(), "Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
      }

      void topic_callback_3(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const {
        const auto &info = msg->info;
        RCLCPP_INFO(this->get_logger(),
                    "Map: %ux%u, res=%.3f, origin=(%.2f, %.2f, %.2f)",
                    info.width, info.height, info.resolution,
                    info.origin.position.x, info.origin.position.y, info.origin.position.z);

        if (!msg->data.empty()) {
          RCLCPP_INFO(this->get_logger(), "Data[0]: %d", static_cast<int>(msg->data[0]));
        } else {
          RCLCPP_WARN(this->get_logger(), "Map data[] is empty");
        }
      }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_3;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
