#ifndef YHS_AUTOWARE_INTERFACE__YHS_AUTOWARE_INTERFACE_NODE_H__
#define YHS_AUTOWARE_INTERFACE__YHS_AUTOWARE_INTERFACE_NODE_H__

#include "vehicle_info_util/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"
#include "yhs_can_msgs/msg/ctrl_command.hpp"
#include "yhs_can_msgs/msg/ctrl_feeback.hpp"
#include "yhs_can_msgs/msg/io_feedback.hpp"
#include "yhs_can_msgs/msg/io_command.hpp"

using namespace std::chrono_literals;

namespace yhs_autoware_interface
{

class YHSAutowareInterfaceNode : public rclcpp::Node
{
  public:
    YHSAutowareInterfaceNode(const rclcpp::NodeOptions & nodeOptions);
    /* 主循环回调 */
    void timerCallback();

  private:
    std::string baseFrame_;                      /* 发布的信息Frame ID */
    vehicle_info_util::VehicleInfo vehicleInfo_; /* 车辆参数 */
    rclcpp::TimerBase::SharedPtr timer_;         /* 主循环 */
    uint8_t waitForMessageFlag_ = 0;             /* 等待Autoware和YHS话题发布的FLAG */
    /* YHS信息话题订阅与发布者 */
    rclcpp::Subscription<yhs_can_msgs::msg::CtrlFeeback>::SharedPtr ctrlFeedbackSubscriber_;
    rclcpp::Subscription<yhs_can_msgs::msg::IoFeedback>::SharedPtr ioFeedbackSubscriber_;
    rclcpp::Publisher<yhs_can_msgs::msg::CtrlCommand>::SharedPtr ctrlCommandPublisher_;
    rclcpp::Publisher<yhs_can_msgs::msg::IoCommand>::SharedPtr ioCommandPublisher_;
    /* Autoware信息话题订阅与发布者 */
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr ackermannControlSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gearSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turnIndicatorsSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engageSubscriber_;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr vehicleEmergencySubscriber_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr controlModePublisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocityPublisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steeringPublisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gearPublisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turnIndicatorsPublisher_;
    /* 最后一次收到的话题存储 */
    yhs_can_msgs::msg::CtrlFeeback ctrlFeedbackMessage_;
    yhs_can_msgs::msg::IoFeedback ioFeedbackMessage_;
    autoware_auto_control_msgs::msg::AckermannControlCommand ackermannControlMessage_;
    autoware_auto_vehicle_msgs::msg::GearCommand gearCommandMessage_;
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turnIndicatorsCommandMessage_;
    autoware_auto_vehicle_msgs::msg::Engage engageMessage_;
    tier4_vehicle_msgs::msg::VehicleEmergencyStamped vehicleEmergencyStampedMessage_;

    /**
     * 发布Autoware的ControlModeReport话题
     *
     */
    void toAutowareControlModeReportMessage();

    /**
     * 发布Autoware的VelocityReport话题
     *
     */
    void toAutowareVelocityReportMessage();

    /**
     * 发布Autoware的SteeringReport话题
     *
     */
    void toAutowareSteeringReportMessage();

    /**
     * 发布Autoware的GearReport话题
     *
     */
    void toAutowareGearReportMessage();

    /**
     * 发布Autoware的TurnIndicatorsReport话题
     *
     */
    void toAutowareTurnIndicatorsReportMessage();


    /**
     * 发布YHS的CtrlCommand话题
     * 
     */
    void toYHSCtrlCommand();

    /**
     * 发布YHS的IoCommand话题
     * 
     */
    void toYHSIoCommand();
};

}  // namespace yhs_autoware_interface

#endif