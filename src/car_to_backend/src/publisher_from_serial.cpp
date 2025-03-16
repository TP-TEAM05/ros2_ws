#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <fstream>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

void loadParams(std::string& dev_name){
  std::ifstream file("/home/ubuntu/ros2_ws/src/car_to_backend/src/serial_read_config");

  if (!file){
      perror("Serial config file not found");
  }

  file >> dev_name;
  file.close();
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("serial_pub"), 
        sockfd_(-1),
        fd_(-1)
  {

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_from_serial", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));

    std::string device;
    loadParams(device);

    // Setup serial
    // Open the serial port "/dev/ttyACM0" for reading
    fd_ = open(device.c_str(), O_RDONLY);
    if (fd_ == -1) {
        perror("Error opening serial port");
        
    }

    // Configure the serial port
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(fd_);
        
    }

    // Set the baud rate to 115200
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

  ~MinimalPublisher()
  {
    if (fd_ != 1)
    {
      close(fd_);
    }
    //RCLCPP_INFO(this->get_logger(), "Port closed!");
  }

private:
  void timer_callback()
  //void topic_callback()
  {
    // Read and print bytes from the serial port
    char buffer[256] = {0};
    
    ssize_t bytesRead;
    
    bytesRead = read(fd_, buffer, sizeof(buffer));
    if (bytesRead > 0) {
      //printf("%s\n", buffer);
    } else if (bytesRead < 0) {
      perror("Error reading from serial port");
    }
    std::string buf_str(buffer);

    auto message = std_msgs::msg::String();
    message.data = buf_str;

    // RCLCPP_INFO(this->get_logger(), "DATA from RPI PICO: '%s'", buffer);
    
    publisher_->publish(message);
    //usleep(9500);
    ioctl(fd_, TCFLSH, 2);  // Drops the data on serial port
  }

  int sockfd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int fd_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
