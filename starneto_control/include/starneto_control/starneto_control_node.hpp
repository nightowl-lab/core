#ifndef STARNETO_CONTROL_NODE_H
#define STARNETO_CONTROL_NODE_H

#include <string>
#include <sstream>
#include <chrono>
#include <memory>
#include <ctype.h>
#include <rclcpp/rclcpp.hpp>

#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gtimu.hpp"
#include "starneto_msgs/msg/gpgga.hpp"
#include "starneto_msgs/msg/pos320_nav.hpp"
#include "starneto_control/tools.hpp"

#include<serial/serial.h>

using namespace std::chrono_literals;

#define BUFFER_SIZE 1024 * 5
#define GPS_TIME_TO_UTC 315936000

namespace starneto_control
{

class StarnetoControlNode : public rclcpp::Node
{
public:
    explicit StarnetoControlNode(const rclcpp::NodeOptions & nodeOptions);
    ~StarnetoControlNode();

    void timeCallback(void);
    void serialInit(void);
    void parseData(void);

private:
    /* data */
    int node_rate_;
    int baudrate_;
    std::string buffer_;
    std::vector<int> comma_pos;
    std::vector<std::string> substrs;
    std::string port_;
    std::string gnss_frame_;
    std::string imu_frame_;
    serial::Serial serial_;

    /* 主循环 */
    bool timesync_flag;
    rclcpp::TimerBase::SharedPtr timer_; 
    starneto_msgs::msg::Gpfpd gnss_;
    starneto_msgs::msg::Gtimu imu_;
    starneto_msgs::msg::Gpgga gpgga_;
    rclcpp::Publisher<starneto_msgs::msg::Gpfpd>::SharedPtr gpfpdPublisher_;
    rclcpp::Publisher<starneto_msgs::msg::Gtimu>::SharedPtr gtimuPublisher_;
    rclcpp::Publisher<starneto_msgs::msg::Gpgga>::SharedPtr gpggaPublisher_;
};

}

#endif