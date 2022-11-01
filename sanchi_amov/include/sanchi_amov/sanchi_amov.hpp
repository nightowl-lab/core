#ifndef SANCHI_AMOV_H
#define SANCHI_AMOV_H

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/nav_sat_fix.hpp>
#include<sensor_msgs/msg/magnetic_field.hpp>
#include<chrono>
#include<deque>
#include<string>
#include<sstream>
#include<functional>
#include<stdexcept>
#include<boost/assert.hpp>
#include<boost/asio.hpp>
#include<boost/asio/serial_port.hpp>
#include<eigen3/Eigen/Geometry>

// #include<tf2/LinearMath/Vector3.h>
// #include<tf2/LinearMath/Matrix3x3.h>
// #include<tf2/LinearMath/Quaternion.h>
//#include<serial/serial.h>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

using namespace std::chrono_literals;

namespace imu_driver
{

static float d2f_acc(uint8_t a[2]);
static float d2f_gyro(uint8_t a[2]);
static float d2f_mag(uint8_t a[2]);
static float d2f_euler(uint8_t a[2]);
static double d2f_latlon(uint8_t a[4]);
static double d2f_gpsvel(uint8_t a[2]);
static float d2ieee754(uint8_t a[4]);
int uart_set(int fd, int baude, int c_flow, int bits, char parity, int stop);

class ImuDriverNode : public rclcpp::Node
{
public:
    explicit ImuDriverNode(const rclcpp::NodeOptions & options);
    ~ImuDriverNode();

    void timeCallback(void);
    void serial_setup(void);
    void parse_data(void);
private:
    int baudrate_;
    double delay_;
    std::string model_;
    std::string port_;
    std::string frame_id_;

    /* 主循环 */
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPublisher_;
};

}




#endif