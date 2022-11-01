#ifndef YHS_CAN_CONTROL__DIAGNOSTIC_Diagnostic_H__
#define YHS_CAN_CONTROL__DIAGNOSTIC_Diagnostic_H__

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "yhs_can_msgs/msg/battery_flag_feedback.hpp"
#include "yhs_can_msgs/msg/diagnostic_feedback.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace yhs_can_control
{

typedef struct
{
    bool isOnline;                    /* 底盘是否在线 */
} StatusDiagnostic;


class DiagnosticPublisher
{
  public:
    DiagnosticPublisher(rclcpp::Node & node);

    void updateBMSFlagMessage(yhs_can_msgs::msg::BatteryFlagFeedback & msg);

    void updateOnlineStatus(bool isOnline);

    void updateChassisStatus(yhs_can_msgs::msg::DiagnosticFeedback & msg);

    /* ROS回调 */
    void timerCallback();

    /* 初始化话题频率诊断实例 */
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> ctrlFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> leftWheelFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> rightWheelFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> ioFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> odometryFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> batteryInformationFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> batteryFlagFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> wheelEncoderFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> diagnosticFeedbackDiagnostic;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> isOnlineDiagnostic;

  private:
    rclcpp::Node & node_;
    rclcpp::TimerBase::SharedPtr timer_; /* 主循环 */
    /* 话题频率限制 */
    double minFrequencyFor10HZ_; /* 10HZ话题发布最小频率 */
    double maxFrequencyFor10HZ_; /* 10HZ话题发布最大频率 */
    double minFrequencyFor20HZ_; /* 20HZ话题发布最小频率 */
    double maxFrequencyFor20HZ_; /* 20HZ话题发布最大频率 */
    double minFrequencyFor100HZ_; /* 100HZ话题发布最小频率 */
    double maxFrequencyFor100HZ_; /* 100HZ话题发布最大频率 */
    StatusDiagnostic status_; /* 底盘常见状态诊断 */
    diagnostic_updater::Updater updater_; /* 诊断信息更新器 */
    yhs_can_msgs::msg::DiagnosticFeedback lastDiagnosticMessage_; /* 最后一次收到的底盘诊断信息 */
    yhs_can_msgs::msg::BatteryFlagFeedback lastBatteryFlagMessage_; /* 最后一次收到的BMS标志位信息 */

    /**
     * 底盘常见状态信息诊断
     *
     * @param stat
     */
    void statusDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat);
    
    /**
     * 标志位诊断
     *
     * @param stat
     */
    void chassisDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat);

    /**
     * BMS诊断
     *
     * @param stat
     */
    void bmsDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

}  // namespace yhs_can_control
#endif