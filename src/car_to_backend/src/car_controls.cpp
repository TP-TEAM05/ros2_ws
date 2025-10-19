#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Config file serial_config
using std::placeholders::_1;
using namespace std::chrono_literals;

void loadParams(std::string& dev_name){
  std::ifstream file("/home/ubuntu/ros2_ws/src/car_to_backend/src/serial_write_config");

  if (!file){
      perror("Serial config file not found");
  }

  file >> dev_name;
  file.close();
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("car_controls"), 
        sockfd_(-1),
        fd_(-1)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "car_controls", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    std::string device;
    loadParams(device);

    // Setup serial
    fd_ = open(device.c_str(), O_WRONLY);
    if (fd_ == -1) {
        perror("Error opening serial port");
    }

    // Configure the serial port
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(fd_);
    }

    // Set the baud rate to 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Set other serial port parameters
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Set input mode (non-canonical, no echo)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Apply the configuration
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        close(fd_);
    }
  }

  ~MinimalSubscriber()
  {
    if (fd_ != 1)
    {
      close(fd_);
    }
    RCLCPP_INFO(this->get_logger(), "Port closed!");
  }

private:
  void topic_callback(const std_msgs::msg::String &msg)
  {
    std::vector<uint8_t> buffer;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    std::string convert;

    ssize_t bytesWrite = write(fd_, msg.data.c_str(), strlen(msg.data.c_str()));
    if (bytesWrite == -1){
        perror("Error writing to serial port");
        close(fd_);
    }
  }

  int sockfd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int fd_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}