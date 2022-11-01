#include "yhs_can_control/diagnostic_publisher.hpp"


namespace yhs_can_control
{

DiagnosticPublisher::DiagnosticPublisher(rclcpp::Node & node) : node_(node), updater_(&node)
{
    using diagnostic_updater::FrequencyStatusParam;
    using diagnostic_updater::TopicDiagnostic;
    using diagnostic_updater::TimeStampStatusParam;
    /* 初始化常量 */
    minFrequencyFor10HZ_ = 9;
    maxFrequencyFor10HZ_ = 11;
    minFrequencyFor20HZ_ = 19;
    maxFrequencyFor20HZ_ = 21;
    minFrequencyFor100HZ_ = 99; 
    maxFrequencyFor100HZ_ = 101; 
    /* 初始化诊断模块 */
    updater_.setHardwareID("yhs_can_control_node");
    updater_.add("bms", this, &DiagnosticPublisher::bmsDiagnosticTask);
    updater_.add("chassis", this, &DiagnosticPublisher::chassisDiagnosticTask);
    updater_.add("status", this, &DiagnosticPublisher::statusDiagnosticTask);
    /* 初始化话题频率诊断 */
    ctrlFeedbackDiagnostic.reset(new TopicDiagnostic("output/ctrl_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    leftWheelFeedbackDiagnostic.reset(new TopicDiagnostic("output/left_wheel_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    rightWheelFeedbackDiagnostic.reset(new TopicDiagnostic("output/right_wheel_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    ioFeedbackDiagnostic.reset(new TopicDiagnostic("output/io_feedback", updater_, FrequencyStatusParam(&minFrequencyFor20HZ_, &maxFrequencyFor20HZ_), TimeStampStatusParam()));
    odometryFeedbackDiagnostic.reset(new TopicDiagnostic("output/odometry_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    batteryInformationFeedbackDiagnostic.reset(new TopicDiagnostic("output/battery_information_feedback", updater_, FrequencyStatusParam(&minFrequencyFor10HZ_, &maxFrequencyFor10HZ_), TimeStampStatusParam()));
    batteryFlagFeedbackDiagnostic.reset(new TopicDiagnostic("output/battery_flag_feedback", updater_, FrequencyStatusParam(&minFrequencyFor10HZ_, &maxFrequencyFor10HZ_), TimeStampStatusParam()));
    wheelEncoderFeedbackDiagnostic.reset(new TopicDiagnostic("output/wheel_encoder_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    diagnosticFeedbackDiagnostic.reset(new TopicDiagnostic("output/diagnostic_feedback", updater_, FrequencyStatusParam(&minFrequencyFor100HZ_, &maxFrequencyFor100HZ_), TimeStampStatusParam()));
    isOnlineDiagnostic.reset(new TopicDiagnostic("output/is_online", updater_, FrequencyStatusParam(&minFrequencyFor10HZ_, &maxFrequencyFor10HZ_), TimeStampStatusParam()));
}

void DiagnosticPublisher::updateBMSFlagMessage(yhs_can_msgs::msg::BatteryFlagFeedback & msg)
{
    lastBatteryFlagMessage_ = msg;
}

void DiagnosticPublisher::updateOnlineStatus(bool isOnline)
{
    status_.isOnline = isOnline;
}

void DiagnosticPublisher::updateChassisStatus(yhs_can_msgs::msg::DiagnosticFeedback & msg)
{
    lastDiagnosticMessage_ = msg;
}

void DiagnosticPublisher::statusDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("online", status_.isOnline);
    if (!status_.isOnline) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Vehicle Offline!");
        RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "Vehicle Offline!");
    }
}

void DiagnosticPublisher::bmsDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    if (!status_.isOnline) return;
    stat.add("percent", (int)lastBatteryFlagMessage_.percent);
    stat.add("single_over_voltage", lastBatteryFlagMessage_.single_over_voltage);
    stat.add("single_under_voltage", lastBatteryFlagMessage_.single_under_voltage);
    stat.add("over_voltage", lastBatteryFlagMessage_.over_voltage);
    stat.add("under_voltage", lastBatteryFlagMessage_.under_voltage);
    stat.add("charge_over_temperature", lastBatteryFlagMessage_.charge_over_temperature);
    stat.add("charge_under_temperature", lastBatteryFlagMessage_.charge_under_temperature);
    stat.add("charge_over_current", lastBatteryFlagMessage_.charge_over_current);
    stat.add("discharge_over_temperature", lastBatteryFlagMessage_.discharge_over_temperature);
    stat.add("discharge_under_temperature", lastBatteryFlagMessage_.discharge_under_temperature);
    stat.add("discharge_over_current", lastBatteryFlagMessage_.discharge_over_current);
    stat.add("short_circuit", lastBatteryFlagMessage_.short_circuit);
    stat.add("ic_error", lastBatteryFlagMessage_.ic_error);
    stat.add("mos_locked", lastBatteryFlagMessage_.mos_locked);
    stat.add("max_temperature_in_battry", lastBatteryFlagMessage_.max_temperature_in_battry);
    stat.add("min_temperature_in_battry", lastBatteryFlagMessage_.min_temperature_in_battry);
    if (lastBatteryFlagMessage_.percent < 7) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Very Low battery");
    } else if (lastBatteryFlagMessage_.percent < 10) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low battery");
    } else if (lastBatteryFlagMessage_.single_over_voltage || lastBatteryFlagMessage_.single_under_voltage || lastBatteryFlagMessage_.over_voltage || lastBatteryFlagMessage_.under_voltage) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery Fault");
    } else if (lastBatteryFlagMessage_.charge_over_temperature || lastBatteryFlagMessage_.charge_under_temperature || lastBatteryFlagMessage_.charge_over_current) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Charging Stop");
    } else if (lastBatteryFlagMessage_.discharge_over_temperature || lastBatteryFlagMessage_.discharge_under_temperature || lastBatteryFlagMessage_.discharge_over_current) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Discharging Stop");
    } else if (lastBatteryFlagMessage_.short_circuit || lastBatteryFlagMessage_.mos_locked) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "BMS Fault");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
    }
}

void DiagnosticPublisher::chassisDiagnosticTask(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    if (!status_.isOnline) return;
    stat.add("fault_level", (int)lastDiagnosticMessage_.fault_level);
    stat.add("auto_ctrl_command_can_error", lastDiagnosticMessage_.auto_ctrl_command_can_error);
    stat.add("auto_io_command_can_error", lastDiagnosticMessage_.auto_io_command_can_error);
    stat.add("eps_offline", lastDiagnosticMessage_.eps_offline);
    stat.add("eps_fault", lastDiagnosticMessage_.eps_fault);
    stat.add("eps_mos_over_temperature", lastDiagnosticMessage_.eps_mos_over_temperature);
    stat.add("eps_warning", lastDiagnosticMessage_.eps_warning);
    stat.add("eps_running_fault", lastDiagnosticMessage_.eps_running_fault);
    stat.add("eps_over_current", lastDiagnosticMessage_.eps_over_current);
    stat.add("ecu_fault", lastDiagnosticMessage_.ecu_fault);
    stat.add("ehb_offline", lastDiagnosticMessage_.ehb_offline);
    stat.add("ehb_running_mode_fault", lastDiagnosticMessage_.ehb_running_mode_fault);
    stat.add("ehb_disabled", lastDiagnosticMessage_.ehb_disabled);
    stat.add("ehb_angle_sensor_fault", lastDiagnosticMessage_.ehb_angle_sensor_fault);
    stat.add("ecu_over_temperature", lastDiagnosticMessage_.ecu_over_temperature);
    stat.add("ehb_power_fault", lastDiagnosticMessage_.ehb_power_fault);
    stat.add("ehb_sensor_reliability_error", lastDiagnosticMessage_.ehb_sensor_reliability_error);
    stat.add("ehb_motor_fault", lastDiagnosticMessage_.ehb_motor_fault);
    stat.add("ehb_oil_pressure_sensor_fault", lastDiagnosticMessage_.ehb_oil_pressure_sensor_fault);
    stat.add("ehb_oil_tubing_fault", lastDiagnosticMessage_.ehb_oil_tubing_fault);
    stat.add("driver_mcu_offline", lastDiagnosticMessage_.driver_mcu_offline);
    stat.add("driver_mcu_over_temperature", lastDiagnosticMessage_.driver_mcu_over_temperature);
    stat.add("driver_mcu_over_voltage", lastDiagnosticMessage_.driver_mcu_over_voltage);
    stat.add("driver_mcu_under_voltage", lastDiagnosticMessage_.driver_mcu_under_voltage);
    stat.add("driver_mcu_short_circuit", lastDiagnosticMessage_.driver_mcu_short_circuit);
    stat.add("driver_mcu_emergency", lastDiagnosticMessage_.driver_mcu_emergency);
    stat.add("driver_mcu_hall_sensor_fault", lastDiagnosticMessage_.driver_mcu_hall_sensor_fault);
    stat.add("driver_mcu_mos_fault", lastDiagnosticMessage_.driver_mcu_mos_fault);
    stat.add("driver_mcu_out_of_control_fault", lastDiagnosticMessage_.driver_mcu_out_of_control_fault);
    stat.add("bms_offline", lastDiagnosticMessage_.bms_offline);
    stat.add("emergency", lastDiagnosticMessage_.emergency);
    stat.add("rc_power_off", lastDiagnosticMessage_.rc_power_off);
    stat.add("rc_offline", lastDiagnosticMessage_.rc_offline);
    if (lastDiagnosticMessage_.fault_level == 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Some error occurred.");
    }
}

}  // namespace yhs_can_control