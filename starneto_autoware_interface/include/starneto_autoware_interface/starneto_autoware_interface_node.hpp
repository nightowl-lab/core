#ifndef STARNETO_AUTOWARE_INTERFACES__NODE_H
#define STARNETO_AUTOWARE_INTERFACES__NODE_H

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gtimu.hpp"
#include "starneto_msgs/msg/gpgga.hpp"
#include "starneto_msgs/msg/pos320_nav.hpp"
#include "starneto_autoware_interface/tool.hpp"

#define GPS_HDOP_TO_COV 1.5f
#define RTK_HDOP_TO_COV 0.02f

using namespace std::chrono_literals;

namespace starneto_autoware_interface
{

class StarnetoAutowareInterfaceNode : public rclcpp::Node
{
public:
    explicit StarnetoAutowareInterfaceNode(const rclcpp::NodeOptions& nodeOptions);

    void timerCallback();
private:
    std::string baseFrame_;
    rclcpp::TimerBase::SharedPtr timer_;         /* 主循环 */

    rclcpp::Subscription<starneto_msgs::msg::Gpfpd>::SharedPtr gpfpdSubscriber_;
    rclcpp::Subscription<starneto_msgs::msg::Gtimu>::SharedPtr gtimuSubscriber_;
    rclcpp::Subscription<starneto_msgs::msg::Gpgga>::SharedPtr gpggaSubscriber_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPublisher_;
    rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr navPvtPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;

    starneto_msgs::msg::Gpfpd gpfpdMessage_;
    starneto_msgs::msg::Gtimu gtimuxMessage_;
    starneto_msgs::msg::Gpgga gpggaMessage_;

    void toAutowareNavSatFixMessage();
    void toAutowareNavPvtMessage();
    void toAutowareImuMessage();
};

}

#endif
