#include <functional>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <iostream>
#include <cstring>
#include <fstream>

// Include the nlohmann/json header file
#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"


using json = nlohmann::json;  
using std::placeholders::_1;
using namespace std::chrono_literals;

bool interfaceIsUp(const std::string& interfaceName) {
  std::ifstream ifs("/sys/class/net/" + interfaceName + "/operstate");
  if (!ifs) {
    return false;
  }

  std::string status;
  ifs >> status;

  return (status == "up");
}

void loadParams(std::string& ip, std::string& port) {
  std::ifstream file("/ros2_ws2/src/tp5_ros/src/udp_server_config");

  if (!file) {
    perror("UDP server config file not found");
  }

  file >> ip;
  file >> port;
  file.close();
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("udp_pub"),
        sockfd_(-1)
  {
    RCLCPP_INFO(this->get_logger(), "Started");
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("car_controls", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));

    // Setup UDP socket
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      RCLCPP_INFO(this->get_logger(), "Cannot create socket");
    } else {
      RCLCPP_INFO(this->get_logger(), "Socket created!");
    }

    // Load params
    std::string ip, port;
    loadParams(ip, port);
    RCLCPP_INFO(this->get_logger(), "[SERVER CONFIG] IP: %s, PORT: %s", ip.c_str(), port.c_str());

    // Clear out both addresses properly
    memset(&servaddr_, 0, sizeof(servaddr_));
    memset(&cliaddr_, 0, sizeof(cliaddr_));

    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(std::stoi(port));  // server_port
    servaddr_.sin_addr.s_addr = inet_addr(ip.c_str());  // server_ip

    // Set timeout options to handle no new traffic issues
    struct timeval tv;
    tv.tv_sec = 0;   // 0 seconds
    tv.tv_usec = 10; // 10 microseconds
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    // Bind
    if (bind(sockfd_, (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
      RCLCPP_INFO(this->get_logger(), "Binding error!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Binded!");
    }

    memset(buff_, 0, 256);  
    memcpy(buff_, "<0,0,0>", 8);
  }

  ~MinimalPublisher()
  {
    // Close socket properly
    if (sockfd_ != -1) {
      close(sockfd_);
    }
  }

private:
  void timer_callback()
  {
    char buffer[256] = {0}; 
    socklen_t len = sizeof(cliaddr_);

    // Receive from UDP client 
    // (important to pass &len, not (socklen_t *)sizeof(cliaddr_))
    ssize_t nbytes = recvfrom(sockfd_, buffer, 256, 0,
                              reinterpret_cast<struct sockaddr*>(&cliaddr_),
                              &len);

    if (nbytes < 0) {
      // Could be a timeout, EAGAIN, or other error
      RCLCPP_INFO(this->get_logger(), "TIMEOUT or error receiving UDP.");
      return;
    }

    std::string buf_str(buffer);
    
    // Parse JSON and extract the "message" field
    std::string extracted_message;
    try {
      json j = json::parse(buf_str);
      // Check if the keys exist
      if(j.contains("updateVehicleDecision") && j["updateVehicleDecision"].contains("message")){
        extracted_message = j["updateVehicleDecision"]["message"].get<std::string>();
      } else {
        RCLCPP_ERROR(this->get_logger(), "JSON does not contain expected keys.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
    }

    std::string speed_str;
    std::string angle_str;
    speed_str = extracted_message.substr(1, extracted_message.find(","));
    angle_str = extracted_message.substr(extracted_message.find(",") + 1, extracted_message.find(">"));
  
    // Publish only the extracted message part
    auto message = ackermann_msgs::msg::AckermannDrive();
    message.speed = std::stof(speed_str);
    message.steering_angle = std::stof(angle_str);
    message.steering_angle_velocity = 0.0;
    message.acceleration = 0.0;
    message.jerk = 0.0;
    publisher_->publish(message);
    //RCLCPP_INFO(this->get_logger(), "[UDP_PUB] Published message: %s", extracted_message.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
  int sockfd_;
  struct sockaddr_in servaddr_, cliaddr_;
  char buff_[256];
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
