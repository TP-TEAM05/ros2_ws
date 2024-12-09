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

using namespace std::chrono_literals;

double globalLongitude = 0.0;
double globalLatitude = 0.0;
double globalHorizontalAccuracy = 0.0;

class GpsPublisher : public rclcpp::Node
{
public:
    GpsPublisher() : Node("gps_pub")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("gps", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&GpsPublisher::timer_callback, this));
        std::cout << "Publisher start" << std::endl;
    }

    ~GpsPublisher()
    {
        close(ser);
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String message;
        // write message in format "<lat,lon,hacc> to message.data"
        std::stringstream ss;
        ss << "<" << globalLatitude << "," << globalLongitude << "," << globalHorizontalAccuracy << ">";
        message.data = ss.str();
        publisher_->publish(message);
        std::cout << "<" << globalLatitude << "," << globalLongitude << "," << globalHorizontalAccuracy << ">" << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// Configuration and Initialization
std::string tty = "/dev/ttyACM1";
int ser;

// Global variables
std::vector<uint8_t> rawMessage(42);
int globalGroundSpeed = 0;
int globalCourse = 0;
bool print_hpposllh = false;
bool print_velned = false;
bool ntripEnabled = true;

// Function declarations
void configureSerial();
void gps_receiver_thread();
void ntrip_client_thread();
void parse_msg();
void parse_velned();
void parse_hpposllh();
std::string base64_encode(const unsigned char *data, size_t input_length);

int main(int argc, char **argv)
{
    std::thread gpsThread(gps_receiver_thread);
    std::cout << "GPS Thread start" << std::endl;

    if (ntripEnabled)
    {
        std::thread ntripThread(ntrip_client_thread);
        std::cout << "NTRIP Thread start" << std::endl;
        ntripThread.join();
    }

    gpsThread.join();

    rclcpp::init(argc, argv);
    auto gps_publisher = std::make_shared<GpsPublisher>();
    configureSerial();
    rclcpp::spin(gps_publisher);

    rclcpp::shutdown();

    return 0;
}

void configureSerial()
{
    ser = open(tty.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (ser == -1)
    {
        std::cerr << "Failed to open serial port" << std::endl;
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(ser, &tty) != 0)
    {
        std::cerr << "Failed to get serial attributes" << std::endl;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // enable reading and ignore control lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit characters
    tty.c_cflag &= ~PARENB;  // no parity bit
    tty.c_cflag &= ~CSTOPB;  // only need 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

    // setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(ser, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error from tcsetattr" << std::endl;
        return;
    }
}

void gps_receiver_thread()
{
    std::cout << "Hello UBX, let's get the data!" << std::endl;
    int mlcounter = 0;

    while (true)
    {
        int bytesToRead;
        ioctl(ser, FIONREAD, &bytesToRead);

        if (bytesToRead > 0)
        {
            uint8_t lastByte;
            read(ser, &lastByte, 1);

            if (lastByte == 0xb5)
            {
                mlcounter = 0;
                parse_msg();
                rawMessage.clear();
                rawMessage.push_back(lastByte);
                mlcounter += 1;
            }
            else
            {
                if (mlcounter < 42)
                {
                    rawMessage.push_back(lastByte);
                    mlcounter += 1;
                }
            }
        }
    }
    // sleep for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void parse_msg()
{
    if (rawMessage.size() == 42)
    { // Ensure the message has the correct length
        if (rawMessage[3] == 0x12)
        {
            parse_velned(); // Parse velocity and course data
        }
        else if (rawMessage[3] == 0x14)
        {
            parse_hpposllh(); // Parse high precision position data
        }
    }
}

void parse_velned()
{
    if (rawMessage.size() == 42 && rawMessage[3] == 0x12)
    {
        int32_t gSpeed, gSAcc, course, gCAcc;

        std::memcpy(&gSpeed, &rawMessage[26], 4);
        std::memcpy(&gSAcc, &rawMessage[34], 4);
        std::memcpy(&course, &rawMessage[30], 4);
        std::memcpy(&gCAcc, &rawMessage[38], 4);

        // Adjust endianess if necessary
        gSpeed = __builtin_bswap32(gSpeed);
        gSAcc = __builtin_bswap32(gSAcc);
        course = __builtin_bswap32(course);
        gCAcc = __builtin_bswap32(gCAcc);

        if (gSpeed < 50000)
        {
            globalGroundSpeed = gSpeed;
            globalCourse = course;
        }

        if (print_velned)
        {

            std::cout << "# # # # # # # # # # # # # # #" << std::endl;
            std::cout << "NAV_VELNED found" << std::endl;
            std::cout << "Ground speed: " << gSpeed * 3.6 / 10.0 << " km/h" << std::endl;
            std::cout << "Ground speed accuracy: " << gSAcc << std::endl;
            std::cout << "Course: " << course / 100000.0 << std::endl;
            std::cout << "Course accuracy: " << gCAcc / 10000.0 << std::endl;
        }
    }
}

void parse_hpposllh()
{
    if (rawMessage.size() == 42 && rawMessage[3] == 0x14)
    {
        int32_t lon, lat, hellip, hmsl;
        int8_t lonHp, latHp, hHp, hMSLHp;
        int32_t hAcc, vAcc;

        std::memcpy(&lat, &rawMessage[18], 4);
        std::memcpy(&lon, &rawMessage[14], 4);
        std::memcpy(&hellip, &rawMessage[22], 4);
        std::memcpy(&hmsl, &rawMessage[26], 4);
        std::memcpy(&hAcc, &rawMessage[34], 4);
        std::memcpy(&vAcc, &rawMessage[38], 4);

        lonHp = rawMessage[30];
        latHp = rawMessage[31];
        hHp = rawMessage[32];
        hMSLHp = rawMessage[33];

        if (lonHp > 127)
            lonHp -= 256;
        if (latHp > 127)
            latHp -= 256;
        if (hHp > 127)
            hHp -= 256;
        if (hMSLHp > 127)
            hMSLHp -= 256;

        globalLongitude = lon / 10000000.0;
        globalLatitude = lat / 10000000.0;
        globalHorizontalAccuracy = hAcc / 10.0;

        if (print_hpposllh)
        {

            std::cout << "# # # # # # # # # # # # # # #" << std::endl;
            std::cout << "NAV_HPPOSLLH found" << std::endl;
            std::cout << "Lat: " << lat / 10000000.0 << ", Lon: " << lon / 10000000.0 << std::endl;
            std::cout << "Height (ellipsoid): " << hellip << " mm, Height (MSL): " << hmsl << " mm" << std::endl;
            std::cout << "hAcc: " << hAcc / 10.0 << " mm, vAcc: " << vAcc / 10.0 << " mm" << std::endl;
        }
        std::cout << "GPS Found" << std::endl;
    }
}

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string base64_encode(const unsigned char *data, size_t input_length)
{
    std::string encoded_string;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (input_length--)
    {
        char_array_3[i++] = *(data++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++)
                encoded_string += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            encoded_string += base64_chars[char_array_4[j]];

        while ((i++ < 3))
            encoded_string += '=';
    }

    return encoded_string;
}
void ntrip_client_thread()
{
    const char *server = "147.175.80.248";
    const int port = 2101;
    const std::string mountpoint = "SUT1";
    const std::string username = "ublox";
    const std::string password = "ublox143";

    std::string user_pass = username + ":" + password;
    std::string auth = base64_encode(reinterpret_cast<const unsigned char *>(user_pass.c_str()), user_pass.length());

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
        return;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    if (inet_pton(AF_INET, server, &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        close(sock);
        return;
    }

    if (connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "Connection Failed" << std::endl;
        close(sock);
        return;
    }

    std::string request = "GET /" + mountpoint + " HTTP/1.0\r\n";
    request += "User-Agent: NTRIP MyClient\r\n";
    request += "Authorization: Basic " + auth + "\r\n";
    request += "Accept: */*\r\n";
    request += "Connection: close\r\n\r\n";

    send(sock, request.c_str(), request.size(), 0);

    const int bufferSize = 1024;
    char buffer[bufferSize];

    while (int bytesRead = read(sock, buffer, bufferSize))
    {
        if (bytesRead > 0)
        {
            write(ser, buffer, bytesRead); // Write directly to serial port
            std::cout << "Written RTK" << std::endl;
        }
    }

    close(sock);
}
