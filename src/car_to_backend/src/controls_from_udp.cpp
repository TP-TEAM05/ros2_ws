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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// UDP server, use udp_server_config to define UDP client, from where the data comes from
#define PORT 12345
#define IP_ADDR "192.168.20.227"

using std::placeholders::_1;
using namespace std::chrono_literals;

bool interfaceIsUp(const std::string& interfaceName){
    std::ifstream ifs("/sys/class/net/" + interfaceName + "/operstate");
    if (!ifs){
        return false;
    }

    std::string status;
    ifs >> status;

    return (status == "up");
}

void loadParams(std::string& ip, std::string& port){
  std::ifstream file("/home/ubuntu/ros2_ws/src/car_to_backend/src/udp_server_config");

  if (!file){
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
    publisher_ = this->create_publisher<std_msgs::msg::String>("car_controls", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));

    // Setup UDP socket
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        RCLCPP_INFO(this->get_logger(), "Cannot create socket");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Socket created!");
    }

    std::string ip, port;
    loadParams(ip, port);

    memset(&servaddr_, 0, sizeof(servaddr_));
    memset(&servaddr_, 0, sizeof(cliaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(stoi(port)); // server_port
    servaddr_.sin_addr.s_addr = inet_addr(ip.c_str());  // server_ip

    // Set timeout options to handle no new traffic issues
    struct timeval tv;
    tv.tv_sec = 0;  // 1s
    tv.tv_usec = 10;   
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    if (bind(sockfd_, (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0){
        RCLCPP_INFO(this->get_logger(), "Binding error!");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Binded!");
    }

    memset(buff_, 0, 256);  // initializes class buffer
    memcpy(buff_, "<0,0,0>", 8);
  }

  ~MinimalPublisher()
  {
    if (sockfd_ != 1)
    {
      close(sockfd_);
    }
  }

private:
  void timer_callback()
  //void topic_callback()
  {
    char buffer[256] = {0}; // local buffer

    // Set default car controls into local buffer <0,0,0>

    // Read message from UDP client
    recvfrom(sockfd_, buffer, 256, 0, (struct sockaddr *)&cliaddr_, (socklen_t *)sizeof(cliaddr_));

    /*
    if (strlen(buffer) <= 0){ // the buffer is empty -> timeout
        RCLCPP_INFO(this->get_logger(), "TIMEOUT!!!");
        if (interfaceIsUp("wlan0")){
            // timed out due to constant velocity -> resend the stored class buffer
            memcpy(buffer, buff_, 256);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Interface is down!");
            memcpy(buffer, "<0,0,0>", 8);
        }
        // else -> the connection is lost -> stop the car
    }
    else{   // saves the read data from socket to class buffer
      memcpy(buff_, buffer, 256);
    }*/

    std::string buf_str(buffer);

    auto message = std_msgs::msg::String();
    message.data = buf_str;

    RCLCPP_INFO(this->get_logger(), "CONTROLS from UDP '%s'", message.data.c_str());
    
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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
