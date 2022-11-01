#ifndef YHS_CAN_CONTROL__YHS_CAN_CONTROL_NODE_H__
#define YHS_CAN_CONTROL__YHS_CAN_CONTROL_NODE_H__

#include <rclcpp/rclcpp.hpp>

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/bool.hpp"
#include "yhs_can_msgs/msg/battery_flag_feedback.hpp"
#include "yhs_can_msgs/msg/battery_information_feedback.hpp"
#include "yhs_can_msgs/msg/ctrl_command.hpp"
#include "yhs_can_msgs/msg/ctrl_feeback.hpp"
#include "yhs_can_msgs/msg/diagnostic_feedback.hpp"
#include "yhs_can_msgs/msg/io_command.hpp"
#include "yhs_can_msgs/msg/io_feedback.hpp"
#include "yhs_can_msgs/msg/odometry_feedback.hpp"
#include "yhs_can_msgs/msg/wheel_encoder_feedback.hpp"
#include "yhs_can_msgs/msg/wheel_feedback.hpp"
#include "yhs_can_msgs/msg/online_status.hpp"
#include "yhs_can_control/diagnostic_publisher.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace yhs_can_control
{

class YHSCANControlNode : public rclcpp::Node
{
  public:
    YHSCANControlNode(const rclcpp::NodeOptions & nodeOptions);

  private:
    DiagnosticPublisher diagnosticPublisher_; /* 诊断工具 */
    std::map<int, int> counters_;             /* 底盘计数器 */
    bool isOnline_ = false;                   /* 底盘是否在线 */
    std::string baseFrame_;                      /* 发布的信息Frame ID */

    /* ROS回调 */
    void timerCallback();
    void canReceiveCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void ctrlCommandCallback(const yhs_can_msgs::msg::CtrlCommand::SharedPtr msg);
    void ioCommandCallback(const yhs_can_msgs::msg::IoCommand::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_; /* 主循环 */

    /* CAN话题订阅与发布者 */
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr canSendPublisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr canReceiveSubscriber_;

    /* 底盘信息话题订阅与发布者 */
    rclcpp::Publisher<yhs_can_msgs::msg::CtrlFeeback>::SharedPtr ctrlFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::WheelFeedback>::SharedPtr leftWheelFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::WheelFeedback>::SharedPtr rightWheelFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::IoFeedback>::SharedPtr ioFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::OdometryFeedback>::SharedPtr odometryFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::BatteryInformationFeedback>::SharedPtr batteryInformationFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::BatteryFlagFeedback>::SharedPtr batteryFlagFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::WheelEncoderFeedback>::SharedPtr wheelEncoderFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::DiagnosticFeedback>::SharedPtr diagnosticFeedbackPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::OnlineStatus>::SharedPtr isOnlinePublisher_;
    rclcpp::Subscription<yhs_can_msgs::msg::CtrlCommand>::SharedPtr ctrlCommandSubscriber_;
    rclcpp::Subscription<yhs_can_msgs::msg::IoCommand>::SharedPtr ioCommandSubscriber_;
};

}  // namespace yhs_can_control
#endif