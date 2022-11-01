#include "yhs_autoware_interface/yhs_autoware_interface_node.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

namespace yhs_autoware_interface
{

YHSAutowareInterfaceNode::YHSAutowareInterfaceNode(const rclcpp::NodeOptions & nodeOptions) : Node("yhs_autoware_interface", nodeOptions)
{
    /* 读取配置 */
    baseFrame_ = this->declare_parameter("base_frame", "base_link", rcl_interfaces::msg::ParameterDescriptor(), true);
    vehicleInfo_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    /* 订阅和发布话题 */
    ctrlFeedbackSubscriber_ = this->create_subscription<yhs_can_msgs::msg::CtrlFeeback>("input/ctrl_feeback", 10, [&](const yhs_can_msgs::msg::CtrlFeeback::SharedPtr msg) {
        waitForMessageFlag_ |= 0x1;
        ctrlFeedbackMessage_ = *msg;
    });
    ioFeedbackSubscriber_ = this->create_subscription<yhs_can_msgs::msg::IoFeedback>("input/io_feedback", 10, [&](const yhs_can_msgs::msg::IoFeedback::SharedPtr msg) {
        waitForMessageFlag_ |= 0x2;
        ioFeedbackMessage_ = *msg;
    });
    ackermannControlSubscriber_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("input/ackermann_control_command", 10, [&](const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg) {
        ackermannControlMessage_ = *msg;
    });
    gearSubscriber_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>("input/gear_command", 10, [&](const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg) {
        gearCommandMessage_ = *msg;
    });
    turnIndicatorsSubscriber_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("input/turn_indicators_command", 10, [&](const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg) {
        turnIndicatorsCommandMessage_ = *msg;
    });
    engageSubscriber_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::Engage>("input/engage_command", 10, [&](const autoware_auto_vehicle_msgs::msg::Engage::SharedPtr msg) {
        engageMessage_ = *msg;
    });
    vehicleEmergencySubscriber_ = this->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("input/vehicle_emergency_command", 10, [&](const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg) {
        vehicleEmergencyStampedMessage_ = *msg;
    });
    ctrlCommandPublisher_ = this->create_publisher<yhs_can_msgs::msg::CtrlCommand>("output/ctrl_command", 10);
    ioCommandPublisher_ = this->create_publisher<yhs_can_msgs::msg::IoCommand>("output/io_command", 10);
    controlModePublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("output/control_mode_report", 10);
    velocityPublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("output/velocity_report", 10);
    steeringPublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("output/steering_report", 10);
    gearPublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("output/gear_report", 10);
    turnIndicatorsPublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("output/turn_indicators_report", 10);
    /* 初始化主循环 */
    timer_ = this->create_wall_timer(100ms, std::bind(&YHSAutowareInterfaceNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "yhs_autoware_interface_node started.");
}

void YHSAutowareInterfaceNode::timerCallback()
{
    /* 等待节点启动完成 */
    if (waitForMessageFlag_ != 0x03) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "waiting for topics from yhs_can_control");
        return;
    }
    /* 转换发布yhs_can_control话题 */
    toYHSCtrlCommand();
    toYHSIoCommand();
    /* 转换发布Autoware话题 */
    toAutowareControlModeReportMessage();
    toAutowareVelocityReportMessage();
    toAutowareSteeringReportMessage();
    toAutowareGearReportMessage();
    toAutowareTurnIndicatorsReportMessage();
}

void YHSAutowareInterfaceNode::toYHSCtrlCommand()
{
    yhs_can_msgs::msg::CtrlCommand msg;
    msg.header.frame_id = baseFrame_;
    msg.header.stamp = this->get_clock()->now();
    msg.steering = engageMessage_.engage && !vehicleEmergencyStampedMessage_.emergency ? ackermannControlMessage_.lateral.steering_tire_angle : 0;
    msg.velocity = engageMessage_.engage && !vehicleEmergencyStampedMessage_.emergency ? std::abs(ackermannControlMessage_.longitudinal.speed) : 0;
    msg.brake = engageMessage_.engage && !vehicleEmergencyStampedMessage_.emergency ? 0 : 100;
    if (gearCommandMessage_.command == autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE) {
        msg.gear = yhs_can_msgs::msg::CtrlCommand::GEAR_D;
    } else if (gearCommandMessage_.command == autoware_auto_vehicle_msgs::msg::GearCommand::PARK) {
        msg.gear = yhs_can_msgs::msg::CtrlCommand::GEAR_P;
    } else if (gearCommandMessage_.command == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE) {
        msg.gear = yhs_can_msgs::msg::CtrlCommand::GEAR_R;
    } else if (gearCommandMessage_.command == autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL) {
        msg.gear = yhs_can_msgs::msg::CtrlCommand::GEAR_N;
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Autoware sent gear command which it cannot be executed by yhs_can_control.");
        msg.gear = yhs_can_msgs::msg::CtrlCommand::GEAR_DISABLE;
    }
    ctrlCommandPublisher_->publish(msg);
}

void YHSAutowareInterfaceNode::toYHSIoCommand()
{
    yhs_can_msgs::msg::IoCommand msg;
    msg.header.frame_id = baseFrame_;
    msg.header.stamp = this->get_clock()->now();
    msg.enable = true;
    if (turnIndicatorsCommandMessage_.command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT) {
        msg.turn_signal = yhs_can_msgs::msg::IoCommand::TURN_SIGNAL_LEFT;
    } else if (turnIndicatorsCommandMessage_.command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT) {
        msg.turn_signal = yhs_can_msgs::msg::IoCommand::TURN_SIGNAL_RIGHT;
    } else {
        msg.turn_signal = yhs_can_msgs::msg::IoCommand::TURN_SIGNAL_OFF;
    }
    ioCommandPublisher_->publish(msg);
}

void YHSAutowareInterfaceNode::toAutowareControlModeReportMessage()
{
    autoware_auto_vehicle_msgs::msg::ControlModeReport msg;
    msg.stamp = this->get_clock()->now();
    if (engageMessage_.engage) {
        if (ctrlFeedbackMessage_.mode == yhs_can_msgs::msg::CtrlFeeback::MODE_AUTO) {
            msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        } else if (ctrlFeedbackMessage_.mode == yhs_can_msgs::msg::CtrlFeeback::MODE_REMOTE) {
            msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
        }
    } else {
        if (ctrlFeedbackMessage_.mode == yhs_can_msgs::msg::CtrlFeeback::MODE_STOP) {
            msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::NOT_READY;
        } else {
            msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
        }
    }
    controlModePublisher_->publish(std::move(msg));
}

void YHSAutowareInterfaceNode::toAutowareVelocityReportMessage()
{
    autoware_auto_vehicle_msgs::msg::VelocityReport msg;
    msg.header.frame_id = baseFrame_;
    msg.header.stamp = this->get_clock()->now();
    msg.longitudinal_velocity = ctrlFeedbackMessage_.velocity;
    if (ctrlFeedbackMessage_.gear == yhs_can_msgs::msg::CtrlFeeback::GEAR_R) {
        msg.longitudinal_velocity = -msg.longitudinal_velocity;
    }
    /* 注意单位 rad/s */
    msg.heading_rate = ctrlFeedbackMessage_.velocity * std::tan(ctrlFeedbackMessage_.steering) / vehicleInfo_.wheel_base_m;
    velocityPublisher_->publish(std::move(msg));
}

void YHSAutowareInterfaceNode::toAutowareSteeringReportMessage()
{
    autoware_auto_vehicle_msgs::msg::SteeringReport msg;
    msg.stamp = this->get_clock()->now();
    msg.steering_tire_angle = ctrlFeedbackMessage_.steering;
    steeringPublisher_->publish(std::move(msg));
}

void YHSAutowareInterfaceNode::toAutowareGearReportMessage()
{
    autoware_auto_vehicle_msgs::msg::GearReport msg;
    msg.stamp = this->get_clock()->now();
    if (ctrlFeedbackMessage_.gear == yhs_can_msgs::msg::CtrlFeeback::GEAR_N) {
        msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL;
    } else if (ctrlFeedbackMessage_.gear == yhs_can_msgs::msg::CtrlFeeback::GEAR_D) {
        msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
    } else if (ctrlFeedbackMessage_.gear == yhs_can_msgs::msg::CtrlFeeback::GEAR_P) {
        msg.report = autoware_auto_vehicle_msgs::msg::GearReport::PARK;
    } else if (ctrlFeedbackMessage_.gear == yhs_can_msgs::msg::CtrlFeeback::GEAR_R) {
        msg.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Gear From yhs_can_msgs out of control!");
        msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
    }
    gearPublisher_->publish(msg);
}

void YHSAutowareInterfaceNode::toAutowareTurnIndicatorsReportMessage()
{
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport msg;
    msg.stamp = this->get_clock()->now();
    if (ioFeedbackMessage_.enable) {
        if (ioFeedbackMessage_.turn_signal == yhs_can_msgs::msg::IoFeedback::TURN_SIGNAL_LEFT) {
            msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
        } else if (ioFeedbackMessage_.turn_signal == yhs_can_msgs::msg::IoFeedback::TURN_SIGNAL_RIGHT) {
            msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
        } else {
            msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
        }
    } else {
        msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    }
    turnIndicatorsPublisher_->publish(msg);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yhs_autoware_interface::YHSAutowareInterfaceNode)