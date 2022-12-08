#ifndef NIGHTOWL_REMOTE_CONTROLLER__NIGHTOWL_REMOTE_CONTROLLER_NODE_H__
#define NIGHTOWL_REMOTE_CONTROLLER__NIGHTOWL_REMOTE_CONTROLLER_NODE_H__

#include "nightowl_remote_controller/h264_encoder.hpp"
#include "nightowl_remote_controller/webrtc.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <yhs_can_msgs/msg/battery_flag_feedback.hpp>

using namespace std::chrono_literals;

namespace nightowl_remote_controller
{

class NightOwlRemoteControllerNode : public rclcpp::Node
{
  public:
    NightOwlRemoteControllerNode(const rclcpp::NodeOptions & nodeOptions);

  private:
    std::unique_ptr<H264Encoder> encoder_;           /* 视频编码器实例 */
    std::unique_ptr<WebRTC> webrtc_;                 /* WebRTC实例 */
    bool isInitalized_ = false;                      /* 是否接受到第一帧图像完成初始化 */
    rclcpp::TimerBase::SharedPtr timer_;             /* 主循环 */
    size_t bitrate_;                                 /* 配置码率 */
    double fps_;                                     /* 配置帧率 */
    std::string signalingServerURL_;                 /* 配置信令服务器URL */
    std::string signalingServerUsername_;            /* 配置信令服务器用户名 */
    std::string signalingServerPassword_;            /* 配置信令服务器密码 */
    std::vector<std::string> iceServers_;            /* 配置ICE服务器 */
    std::vector<std::string> waitingForReportTopic_; /* 还未接受到第一次报告类话题名称 */
    bool allReportTopicReady_ = false;               /* 所有话题就绪 */

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gearReportSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr controlModeReportSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steeringReportSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocityReportSubscriber_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turnIndicatorsSubscriber_;
    rclcpp::Subscription<tier4_external_api_msgs::msg::Emergency>::SharedPtr emergencySubscriber_;
    rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr gateModeSubscriber_;
    rclcpp::Subscription<yhs_can_msgs::msg::BatteryFlagFeedback>::SharedPtr batteryFlagFeedbackSubscriber_;
    rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr autowareStateSubscriber_;

    autoware_auto_vehicle_msgs::msg::GearReport lastGearReport_;
    autoware_auto_vehicle_msgs::msg::ControlModeReport lastControlModeReport_;
    autoware_auto_vehicle_msgs::msg::SteeringReport lastSteeringReport_;
    autoware_auto_vehicle_msgs::msg::VelocityReport lastVelocityReport_;
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport lastTurnIndicatorsReport_;
    tier4_external_api_msgs::msg::Emergency lastEmergency_;
    tier4_control_msgs::msg::GateMode lastGateMode_;
    yhs_can_msgs::msg::BatteryFlagFeedback lastBatteryFlagFeedback_;
    autoware_auto_system_msgs::msg::AutowareState lastAutowareState_;

    /**
     * 订阅各种状态信息
     *
     * @param topicName 话题名称
     * @param subscription 订阅者
     * @param lastMessageContainer 接受到的最后一包数据的容器
     */
    template <typename T>
    void subscribeReport(std::string topicName, typename rclcpp::Subscription<T>::SharedPtr & subscription, T & lastMessageContainer);

    /**
     * 接受摄像头图像
     * @param rawFrame 图像帧
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image);

    /**
     * 主循环回调
     */
    void timerCallback();

    /**
     * 发送车辆报告到客户端
     *
     */
    void sendVehicleReport();
};

}  // namespace nightowl_remote_controller

#endif