#ifndef BAIDU_JOY_CONTROLLER__BAIDU_JOY_CONTROLLER_NODE_H__
#define BAIDU_JOY_CONTROLLER__BAIDU_JOY_CONTROLLER_NODE_H__

#include "baidu_joy_controller/ds4_joy_converter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"
#include "tier4_control_msgs/msg/gate_mode.hpp"
#include "tier4_external_api_msgs/msg/control_command_stamped.hpp"
#include "tier4_external_api_msgs/msg/gear_shift_stamped.hpp"
#include "tier4_external_api_msgs/msg/heartbeat.hpp"
#include "tier4_external_api_msgs/msg/turn_signal_stamped.hpp"
#include "tier4_external_api_msgs/srv/engage.hpp"
#include "tier4_external_api_msgs/srv/set_emergency.hpp"

using namespace std::chrono_literals;

namespace baidu_joy_controller
{

class BaiduJoyControllerNode : public rclcpp::Node
{
  public:
    BaiduJoyControllerNode(const rclcpp::NodeOptions & nodeOptions);
    /* 主循环回调 */
    void timerCallback();

  private:
    rclcpp::TimerBase::SharedPtr timer_; /* 主循环 */
    DS4JoyConverter joy_;                /* 摇杆实例 */
    DS4JoyConverter lastJoy_;            /* 上一次处理摇杆信息实例 */
    float steeringMaxAngle_;             /* 转向最大角度 */
    float forwardRatio_;                 /* 前进加速度系数 */
    float forwardMaxSpeed_;              /* 前进最大速度 */
    float backwardRatio_;                /* 后退加速度系数 */
    float backwardMaxSpeed_;             /* 后退最大速度 */
    uint8_t waitForMessageFlag_ = 0;     /* 等待Autoware话题发布的FLAG */
    bool ledBlinkState_ = false;         /* 手柄LED闪烁时上一次状态 */
    rclcpp::Time ledBlinkLastTime_;      /* 手柄LED状态上一次改变时的时间 */
    bool isFirstJoyMessage_ = true;      /* 是否是第一次收到摇杆信息 */
    rclcpp::Time lastReceivedValidJoy_;  /* 上一次接收到有效的摇杆信息的时间 */
    rclcpp::Duration joyTimeout_;        /* 摇杆信息超时时间 */

    /* Autoware信息话题订阅与发布者 */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocityReportSubscriber_;
    rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr stateSubscriber_;
    rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr controlCommandPublisher_;
    rclcpp::Publisher<tier4_external_api_msgs::msg::GearShiftStamped>::SharedPtr gearShiftPublisher_;
    rclcpp::Publisher<tier4_external_api_msgs::msg::TurnSignalStamped>::SharedPtr turnSignalPublisher_;
    rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr gateModeCommandPublisher_;
    rclcpp::Publisher<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr heartbeatPublisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr vehicleEngagePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joyFeedbackPublisher_;

    /* Autoware相关服务 */
    rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr emergencyStopService_;
    rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr autowareEngageService_;
    rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr clientEngageService_;

    /* 上一次调用的服务存储 */
    tier4_external_api_msgs::srv::SetEmergency::Request::SharedPtr emergencyServiceRequest_;
    tier4_external_api_msgs::srv::Engage::Request::SharedPtr autowareEngageServiceRequest_;
    tier4_external_api_msgs::srv::Engage::Request::SharedPtr clientEngageServiceRequest_;

    /* 上一次发送的话题存储 */
    tier4_external_api_msgs::msg::GearShiftStamped gearMessage_;
    tier4_external_api_msgs::msg::TurnSignalStamped turnSignalMessage_;
    tier4_control_msgs::msg::GateMode gateCommandMessage_;
    autoware_auto_vehicle_msgs::msg::Engage vehicleEngageMessage_;

    /* 上一次收到的话题存储 */
    autoware_auto_vehicle_msgs::msg::VelocityReport velocityMessage_;
    autoware_auto_system_msgs::msg::AutowareState stateMessage_;

    /**
     * 当所有消息和服务准备好后会调用进行一些初始化
     *
     */
    void init();

    /**
     * 调用急停设置服务
     *
     */
    void callEmergencyService();

    /**
     * 调用Autoware Engage指令服务
     *
     */
    void callAutowareEngageService();

    /**
     * 调用Client Engage指令服务
     *
     */
    void callClientEngageService();

    /**
     * 发布AckermannControlCommand类型信息
     *
     */
    void toControlCommandMessage();

    /**
     * 发布GearShiftStamped类型信息
     *
     */
    void toGearShiftMessage();

    /**
     * 发布TurnSignalStamped类型信息
     *
     */
    void toTurnSignalMessage();

    /**
     * 发布GateMode类型信息
     *
     */
    void toGateModeMessage();

    /**
     * 发布Heartbeat类型信息
     *
     */
    void toHeartbeatMessage();

    /**
     * 发布底盘Engage类型信息
     *
     */
    void toVehicleEngageMessage();

    /**
     * 发布急停类型信息
     *
     */
    void toEmergencyService();

    /**
     * 发布Autoware Engage类型信息
     *
     */
    void toAutowareEngageService();

    /**
     * 发布Client Engage类型信息
     *
     */
    void toClientEngageService();

    /**
     * 发布手柄LED信息
     *
     */
    void toJoyLEDMessage();
};

}  // namespace baidu_joy_controller

#endif