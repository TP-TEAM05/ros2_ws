#include <nlohmann/json.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <chrono>
#include <sys/socket.h>
#include <memory>
#include <functional>
#include <fstream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;
using json = nlohmann::ordered_json;

bool is_controlled_by_user;
std::string vin;


void loadParams(std::string &ip, std::string &port, std::string &car_vin, std::string &is_controlled_by_user)
{
  std::ifstream file("/ros2_ws2/src/tp5_ros/src/udp_client_config");

  if (!file)
  {
    perror("UDP client config file not found");
  }

  file >> ip;
  file >> port;
  file >> car_vin;
  file >> is_controlled_by_user;

  file.close();
}


class MinimalSubscriber: public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"),
      sockfd_(-1){
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
      subscription_2 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::QoS(10), std::bind(&MinimalSubscriber::topic_callback_2, this, std::placeholders::_1)); 
      subscription_3 = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10), std::bind(&MinimalSubscriber::topic_callback_3, this, std::placeholders::_1));
      //auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
      {
        RCLCPP_INFO(this->get_logger(), "Cannot create socket");
      }

      std::string ip, port, controlled_by_user;
      loadParams(ip, port, vin, controlled_by_user);

      if (controlled_by_user.empty())
      { 
        // Car is by default not controlled by user
        controlled_by_user = "0";
      }

      is_controlled_by_user = stoi(controlled_by_user);

      memset(&servaddr_, 0, sizeof(servaddr_));
      servaddr_.sin_family = AF_INET;
      servaddr_.sin_port = htons(stoi(port));
      servaddr_.sin_addr.s_addr = inet_addr(ip.c_str());

      dist_lidar = 0.0f;  // Initialize the member variable (not declaring a new one)
    }
    private:
      void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Angle_min: %.3f", msg->angle_min);
          if (!msg->ranges.empty()) {
            RCLCPP_INFO(this->get_logger(), "Range[0]: %.3f", msg->ranges[(int)(msg->range_min+msg->range_max)/2]);
            dist_lidar = msg->ranges[(int)(msg->range_min+msg->range_max)/2];
          } else {
            RCLCPP_WARN(this->get_logger(), "LaserScan ranges[] is empty");
          }
        }

      void topic_callback_3(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extrahuj lineárne zložky rýchlosti
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;

        // Výpočet celkovej rýchlosti (m/s)
        double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        // Vypíš do ROS logu
        RCLCPP_INFO(this->get_logger(), "Current speed: %.2f m/s (vx=%.2f, vy=%.2f, vz=%.2f)", speed, vx, vy, vz);

        // Ulož si rýchlosť pre použitie v JSON správe
        // current_speed_ = static_cast<float>(speed);
      }
      
      void topic_callback_2(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Test: %f", msg->longitude); // %s + c_str()

        
        boost::posix_time::ptime t = boost::posix_time::microsec_clock::universal_time();
        std::string timestamp_iso = to_iso_extended_string(t) + "Z";
        char time_str[50] = {0};
        memcpy(time_str, timestamp_iso.c_str(), timestamp_iso.length());

        json json_to_send;
        json vehicle;

        json_to_send["index"] = 1;
        json_to_send["type"] = "update_vehicle";
        json_to_send["timestamp"] = time_str;

        vehicle["vin"] = vin.c_str();
        vehicle["is_controlled_by_user"] = is_controlled_by_user;
        vehicle["speed"] = std::stof("10"); //should be calculated from odom
        vehicle["longitude"] = msg->longitude;
        vehicle["latitude"] = msg->latitude;
        vehicle["gps_direction"] = msg->position_covariance_type;
        vehicle["front_ultrasonic"] = std::stof("10");
        vehicle["rear_ultrasonic"] = std::stof("10");
        vehicle["front_lidar"] = dist_lidar;
        vehicle["speed_front_left"] = std::stof("1.0");
        vehicle["speed_front_right"] = std::stof("1.0");
        vehicle["speed_rear_left"] = std::stof("1.0");
        vehicle["speed_rear_right"] = std::stof("1.0");
        vehicle["voltage0"] = std::stof("1.0");
        vehicle["voltage1"] = std::stof("1.0");
        vehicle["voltage2"] = std::stof("1.0");
        vehicle["gps_horizontal_accuracy"] = msg->altitude;
        json_to_send["vehicle"] = vehicle;

        std::string str_to_send = json_to_send.dump();

        // Send message to UDP sever
        if (sendto(sockfd_, str_to_send.c_str(), str_to_send.length(), 0, (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0)
        {
          RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "Error sending message");
        }

      }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3;
    
    // Member variables
    int sockfd_;
    struct sockaddr_in servaddr_;
    float dist_lidar;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
