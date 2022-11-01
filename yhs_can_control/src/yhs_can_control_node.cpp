#include "yhs_can_control/yhs_can_control_node.hpp"
#include "yhs_can_control/tool.hpp"

namespace yhs_can_control
{

YHSCANControlNode::YHSCANControlNode(const rclcpp::NodeOptions & nodeOptions) : Node("yhs_can_control", nodeOptions), diagnosticPublisher_(*this)
{
    /* 读取配置 */
    baseFrame_ = this->declare_parameter("base_frame", "base_link", rcl_interfaces::msg::ParameterDescriptor(), true);
    /* 订阅和发布话题 */
    canSendPublisher_ = this->create_publisher<can_msgs::msg::Frame>("output/can_send", 10);
    ctrlFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::CtrlFeeback>("output/ctrl_feedback", 10);
    leftWheelFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::WheelFeedback>("output/left_wheel_feedback", 10);
    rightWheelFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::WheelFeedback>("output/right_wheel_feedback", 10);
    ioFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::IoFeedback>("output/io_feedback", 10);
    odometryFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::OdometryFeedback>("output/odometry_feedback", 10);
    batteryInformationFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::BatteryInformationFeedback>("output/battery_information_feedback", 10);
    batteryFlagFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::BatteryFlagFeedback>("output/battery_flag_feedback", 10);
    wheelEncoderFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::WheelEncoderFeedback>("output/wheel_encoder_feedback", 10);
    diagnosticFeedbackPublisher_ = this->create_publisher<yhs_can_msgs::msg::DiagnosticFeedback>("output/diagnostic_feedback", 10);
    isOnlinePublisher_ = this->create_publisher<yhs_can_msgs::msg::OnlineStatus>("output/is_online", 10);
    canReceiveSubscriber_ = this->create_subscription<can_msgs::msg::Frame>("input/can_receive", 10, std::bind(&YHSCANControlNode::canReceiveCallback, this, std::placeholders::_1));
    ctrlCommandSubscriber_ = this->create_subscription<yhs_can_msgs::msg::CtrlCommand>("input/ctrl_command", 10, std::bind(&YHSCANControlNode::ctrlCommandCallback, this, std::placeholders::_1));
    ioCommandSubscriber_ = this->create_subscription<yhs_can_msgs::msg::IoCommand>("input/io_command", 10, std::bind(&YHSCANControlNode::ioCommandCallback, this, std::placeholders::_1));
    /* 初始化主循环 */
    timer_ = this->create_wall_timer(100ms, std::bind(&YHSCANControlNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "yhs_can_control_node started.");
}

void YHSCANControlNode::timerCallback()
{
    auto msg = yhs_can_msgs::msg::OnlineStatus();
    ADD_HEADER_TO_MESSAGE(msg, baseFrame_, this->get_clock()->now());
    msg.is_online = isOnline_;
    isOnlinePublisher_->publish(std::move(msg));
    diagnosticPublisher_.updateOnlineStatus(isOnline_);
    diagnosticPublisher_.isOnlineDiagnostic->tick(msg.header.stamp);
    /* 恢复离线,如果收到任意一帧又会在线了 */
    isOnline_  = false;
}

void YHSCANControlNode::ctrlCommandCallback(const yhs_can_msgs::msg::CtrlCommand::SharedPtr msg)
{
    can_msgs::msg::Frame frame;
    frame.header.stamp = this->get_clock()->now();
    frame.dlc = 8;
    frame.is_extended = true;
    frame.id = 0x18C4D2D0;
    float velocity = msg->velocity;
	float angle = RAD_TO_DEGREE(msg->steering);
    if (!CHECK_IN_INTERVAL(0.0f, 5.0f, velocity)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Velocity Command Out of range, input [%f], it should be in [0, 5]", velocity);
        velocity = INTERVAL_LIMIT(0.0f, 5.0f, velocity);
    }
    if (!CHECK_IN_INTERVAL(-25.0f, 25.0f, angle)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Steering Command Out of range, input [%f], it should be in [-25, 25]", angle);
        angle = INTERVAL_LIMIT(-25.0f, 25.0f, angle);
    }
    uint16_t sendVelocity = velocity * 1000;
	int16_t sendAngle = angle * 100.0f;
	frame.data[0] |= 0x0f & msg->gear;
	frame.data[0] |= 0xf0 & ((sendVelocity & 0x0f) << 4);
	frame.data[1] = (sendVelocity >> 4) & 0xff;
	frame.data[2] |= 0x0f & (sendVelocity >> 12);
	frame.data[2] |= 0xf0 & ((sendAngle & 0x0f) << 4);
	frame.data[3] = (sendAngle >> 4) & 0xff;
	frame.data[4] |= 0xf0 & ((msg->brake & 0x0f) << 4);
	frame.data[4] |= 0x0f & (sendAngle >> 12);
	frame.data[5] = (msg->brake >> 4) & 0x0f;
    frame.data[6] = (counters_[frame.id]++) << 4;
	frame.data[7] = CALC_BCC(frame.data);
    canSendPublisher_->publish(frame);
}

void YHSCANControlNode::ioCommandCallback(const yhs_can_msgs::msg::IoCommand::SharedPtr msg)
{
    can_msgs::msg::Frame frame;
    frame.header.stamp = this->get_clock()->now();
    frame.dlc = 8;
    frame.id = 0x18C4D7D0;
    frame.is_extended = true;
	frame.data[0] = msg->enable;
    frame.data[1] |= msg->marker_light << 1;
    frame.data[1] |= msg->turn_signal << 2;
	frame.data[2] = msg->horn;
	frame.data[6] = (counters_[frame.id]++) << 4;
	frame.data[7] = CALC_BCC(frame.data);
    canSendPublisher_->publish(frame);
}

void YHSCANControlNode::canReceiveCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    auto time = this->get_clock()->now();
    switch (msg->id) {
        /* 速度控制反馈 */
        case 0x18C4D2EF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::CtrlFeeback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.gear = 0x0f & msg->data[0];
            publishMessage.velocity = (float)((unsigned short)((msg->data[2] & 0x0f) << 12 | msg->data[1] << 4 | (msg->data[0] & 0xf0) >> 4)) / 1000;
            publishMessage.steering = DEGREE_TO_RAD((float)((short)((msg->data[4] & 0x0f) << 12 | msg->data[3] << 4 | (msg->data[2] & 0xf0) >> 4)) / 100.0f);
            publishMessage.brake = (msg->data[4] & 0xf0) >> 4 | (msg->data[5] & 0x0f) << 4;
            publishMessage.mode = (msg->data[5] & 0x30) >> 4;
            ctrlFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.ctrlFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* 左轮反馈 */
        case 0x18C4D7EF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::WheelFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.velocity = (float)((short)(msg->data[1] << 8 | msg->data[0])) / 1000;
            publishMessage.pulse = (int)(msg->data[5] << 24 | msg->data[4] << 16 | msg->data[3] << 8 | msg->data[2]);
            leftWheelFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.leftWheelFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* 右轮反馈 */
        case 0x18C4D8EF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::WheelFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.velocity = (float)((short)(msg->data[1] << 8 | msg->data[0])) / 1000;
            publishMessage.pulse = (int)(msg->data[5] << 24 | msg->data[4] << 16 | msg->data[3] << 8 | msg->data[2]);
            rightWheelFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.rightWheelFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* io反馈 */
        case 0x18C4DAEF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::IoFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.enable = 0x01 & msg->data[0];
            publishMessage.marker_light = 0x02 & msg->data[1];
            publishMessage.turn_signal = (0x0c & msg->data[1]) >> 2;
            publishMessage.brake_light = 0x10 & msg->data[1];
            publishMessage.horn = 0x01 & msg->data[2];
            publishMessage.front_bumper_press = 0x02 & msg->data[3];
            publishMessage.rear_bumper_press = 0x10 & msg->data[3];
            ioFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.ioFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* 里程计反馈 */
        case 0x18C4DEEF: {
            yhs_can_msgs::msg::OdometryFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.cumulative_mileage = (float)((int)(msg->data[3] << 24 | msg->data[2] << 16 | msg->data[1] << 8 | msg->data[0])) / 1000;
            publishMessage.cumulative_angle = DEGREE_TO_RAD((float)((int)(msg->data[7] << 24 | msg->data[6] << 16 | msg->data[5] << 8 | msg->data[4])) / 1000.0f);
            odometryFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.odometryFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* BMS信息反馈 */
        case 0x18C4E1EF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::BatteryInformationFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.voltage = (float)((unsigned short)(msg->data[1] << 8 | msg->data[0])) / 100;
            publishMessage.current = (float)((short)(msg->data[3] << 8 | msg->data[2])) / 100;
            publishMessage.remaining_capacity = (float)((unsigned short)(msg->data[5] << 8 | msg->data[4])) / 100;
            batteryInformationFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.batteryInformationFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* BMS标志位反馈 */
        case 0x18C4E2EF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::BatteryFlagFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.percent = msg->data[0];
            publishMessage.single_over_voltage = 0x01 & msg->data[1];
            publishMessage.single_under_voltage = 0x02 & msg->data[1];
            publishMessage.over_voltage = 0x04 & msg->data[1];
            publishMessage.under_voltage = 0x08 & msg->data[1];
            publishMessage.charge_over_temperature = 0x10 & msg->data[1];
            publishMessage.charge_under_temperature = 0x20 & msg->data[1];
            publishMessage.discharge_over_temperature = 0x40 & msg->data[1];
            publishMessage.discharge_under_temperature = 0x80 & msg->data[1];
            publishMessage.charge_over_current = 0x01 & msg->data[2];
            publishMessage.discharge_over_current = 0x02 & msg->data[2];
            publishMessage.short_circuit = 0x04 & msg->data[2];
            publishMessage.ic_error = 0x08 & msg->data[2];
            publishMessage.mos_locked = 0x10 & msg->data[2];
            publishMessage.charging = 0x20 & msg->data[2];
            publishMessage.max_temperature_in_battry = (float)((short)(msg->data[4] << 4 | msg->data[3] >> 4)) / 10;
            publishMessage.min_temperature_in_battry = (float)((short)((msg->data[6] & 0x0f) << 8 | msg->data[5])) / 10;
            diagnosticPublisher_.updateBMSFlagMessage(publishMessage);
            batteryFlagFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.batteryFlagFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* 驱动器编码器反馈 */
        case 0x18C4DCEF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::WheelEncoderFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.encoder_value = (int)(msg->data[3] << 24 | msg->data[2] << 16 | msg->data[1] << 8 | msg->data[0]);
            wheelEncoderFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.wheelEncoderFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        /* 底盘诊断信息反馈 */
        case 0x18C4EAEF: {
            CHECK_BCC(msg->data);
            CHECK_ALIVE_COUNTER(msg->id, msg->data, counters_);
            yhs_can_msgs::msg::DiagnosticFeedback publishMessage;
            ADD_HEADER_TO_MESSAGE(publishMessage, baseFrame_, time);
            publishMessage.fault_level = 0x0f & msg->data[0];
            publishMessage.auto_ctrl_command_can_error = 0x10 & msg->data[0];
            publishMessage.auto_io_command_can_error = 0x20 & msg->data[0];
            publishMessage.eps_offline = 0x01 & msg->data[1];
            publishMessage.eps_fault = 0x02 & msg->data[1];
            publishMessage.eps_mos_over_temperature = 0x04 & msg->data[1];
            publishMessage.eps_warning = 0x08 & msg->data[1];
            publishMessage.eps_running_fault = 0x10 & msg->data[1];
            publishMessage.eps_over_current = 0x20 & msg->data[1];
            publishMessage.ecu_fault = 0x10 & msg->data[2];
            publishMessage.ehb_offline = 0x20 & msg->data[2];
            publishMessage.ehb_running_mode_fault = 0x40 & msg->data[2];
            publishMessage.ehb_disabled = 0x80 & msg->data[2];
            publishMessage.ehb_angle_sensor_fault = 0x01 & msg->data[3];
            publishMessage.ecu_over_temperature = 0x02 & msg->data[3];
            publishMessage.ehb_power_fault = 0x04 & msg->data[3];
            publishMessage.ehb_sensor_reliability_error = 0x08 & msg->data[3];
            publishMessage.ehb_motor_fault = 0x10 & msg->data[3];
            publishMessage.ehb_oil_pressure_sensor_fault = 0x20 & msg->data[3];
            publishMessage.ehb_oil_tubing_fault = 0x40 & msg->data[3];
            publishMessage.driver_mcu_offline = 0x01 & msg->data[4];
            publishMessage.driver_mcu_over_temperature = 0x02 & msg->data[4];
            publishMessage.driver_mcu_over_voltage = 0x04 & msg->data[4];
            publishMessage.driver_mcu_under_voltage = 0x08 & msg->data[4];
            publishMessage.driver_mcu_short_circuit = 0x10 & msg->data[4];
            publishMessage.driver_mcu_emergency = 0x20 & msg->data[4];
            publishMessage.driver_mcu_hall_sensor_fault = 0x40 & msg->data[4];
            publishMessage.driver_mcu_mos_fault = 0x80 & msg->data[4];
            publishMessage.bms_offline = 0x10 & msg->data[5];
            publishMessage.emergency = 0x20 & msg->data[5];
            publishMessage.rc_power_off = 0x40 & msg->data[5];
            publishMessage.rc_offline = 0x80 & msg->data[5];
            diagnosticPublisher_.updateChassisStatus(publishMessage);
            diagnosticFeedbackPublisher_->publish(publishMessage);
            diagnosticPublisher_.diagnosticFeedbackDiagnostic->tick(publishMessage.header.stamp);
            isOnline_ = true;
            break;
        }

        default:
            break;
    }
}
}  // namespace yhs_can_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yhs_can_control::YHSCANControlNode)