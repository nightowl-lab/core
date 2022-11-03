// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "baidu_joy_controller/baidu_joy_controller_node.hpp"

#include "baidu_joy_controller/tool.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"

namespace baidu_joy_controller
{

BaiduJoyControllerNode::BaiduJoyControllerNode(const rclcpp::NodeOptions & nodeOptions) : Node("baidu_joy_controller", nodeOptions), joyTimeout_(0, 0)
{
    /* 读取参数 */
    steeringMaxAngle_ = this->declare_parameter("steering_max_angle", 25.0f);
    forwardRatio_ = this->declare_parameter("forward_ratio", 1.0f);
    forwardMaxSpeed_ = this->declare_parameter("forward_max_speed", 1.0f);
    backwardRatio_ = this->declare_parameter("backward_ratio", 1.0f);
    backwardMaxSpeed_ = this->declare_parameter("backward_max_speed", 0.5f);
    joyTimeout_ = joyTimeout_.from_seconds(this->declare_parameter("joy_timeout", 100.0f) / 1000);
    /* 初始化各种服务和话题 */
    joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("input/joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() != 6 || msg->buttons.size() != 16) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received Invalid Joy Message, ignoring");
            return;
        }
        lastReceivedValidJoy_ = this->get_clock()->now();
        if (isFirstJoyMessage_) {
            waitForMessageFlag_ |= 0x4;
            lastJoy_.joyCallback(msg);
            joy_.joyCallback(msg);
            isFirstJoyMessage_ = false;
            return;
        }
        joy_.joyCallback(msg);
    });
    velocityReportSubscriber_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("input/velocity_report", 10, [&](const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
        waitForMessageFlag_ |= 0x8;
        velocityMessage_ = *msg;
    });
    stateSubscriber_ = this->create_subscription<autoware_auto_system_msgs::msg::AutowareState>("input/state_report", 10, [&](const autoware_auto_system_msgs::msg::AutowareState::SharedPtr msg) {
        waitForMessageFlag_ |= 0x10;
        stateMessage_ = *msg;
    });
    joyFeedbackPublisher_ = this->create_publisher<sensor_msgs::msg::JoyFeedbackArray>("output/joy_feedback", 10);
    controlCommandPublisher_ = this->create_publisher<tier4_external_api_msgs::msg::ControlCommandStamped>("output/control_command", 10);
    gearShiftPublisher_ = this->create_publisher<tier4_external_api_msgs::msg::GearShiftStamped>("output/shift", 10);
    turnSignalPublisher_ = this->create_publisher<tier4_external_api_msgs::msg::TurnSignalStamped>("output/turn_signal", 10);
    gateModeCommandPublisher_ = this->create_publisher<tier4_control_msgs::msg::GateMode>("output/gate_mode", 10);
    heartbeatPublisher_ = this->create_publisher<tier4_external_api_msgs::msg::Heartbeat>("output/heartbeat", 10);
    vehicleEngagePublisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("output/vehicle_engage", 10);
    emergencyStopService_ = this->create_client<tier4_external_api_msgs::srv::SetEmergency>("service/emergency_stop");
    autowareEngageService_ = this->create_client<tier4_external_api_msgs::srv::Engage>("service/autoware_engage");
    clientEngageService_ = this->create_client<tier4_external_api_msgs::srv::Engage>("service/client_engage");
    emergencyServiceRequest_ = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();
    autowareEngageServiceRequest_ = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
    clientEngageServiceRequest_ = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
    /* 重制LED闪烁时间 */
    ledBlinkState_ = false;
    ledBlinkLastTime_ = this->get_clock()->now();
    /* 初始化主循环 */
    timer_ = rclcpp::create_timer(this, get_clock(), 20ms, std::bind(&BaiduJoyControllerNode::timerCallback, this));
    /* 默认档位 */
    gearMessage_.gear_shift.data == tier4_external_api_msgs::msg::GearShift::NEUTRAL;
}

void BaiduJoyControllerNode::init()
{
    emergencyServiceRequest_->emergency = true;
    clientEngageServiceRequest_->engage = false;
    autowareEngageServiceRequest_->engage = false;
    callEmergencyService();
    callAutowareEngageService();
    callClientEngageService();
}

void BaiduJoyControllerNode::timerCallback()
{
    /* 检查话题和服务就绪 */
    if (emergencyStopService_->service_is_ready()) {
        waitForMessageFlag_ |= 0x01;
    }
    if (autowareEngageService_->service_is_ready()) {
        waitForMessageFlag_ |= 0x02;
    }
    if (clientEngageService_->service_is_ready()) {
        waitForMessageFlag_ |= 0x20;
    }
    if (waitForMessageFlag_ < 0x3F) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for service and message ready...");
        return;
    }
    if (waitForMessageFlag_ == 0x3F) {
        init();
        /* 这样下次callback就不会再调用这里了 */
        waitForMessageFlag_ = 0xFF;
    }
    toJoyLEDMessage();
    toControlCommandMessage();
    toGearShiftMessage();
    toTurnSignalMessage();
    toGateModeMessage();
    toHeartbeatMessage();
    toVehicleEngageMessage();
    toEmergencyService();
    toAutowareEngageService();
    toClientEngageService();
    lastJoy_ = joy_;
}

void BaiduJoyControllerNode::toJoyLEDMessage()
{
    using autoware_auto_system_msgs::msg::AutowareState;
    using tier4_control_msgs::msg::GateMode;
    using tier4_external_api_msgs::msg::TurnSignal;
    if (emergencyServiceRequest_->emergency || !vehicleEngageMessage_.engage) {
        joy_.publishLEDMessage(joyFeedbackPublisher_, 0xFF, 0x00, 0x00);
    } else if (gateCommandMessage_.data == GateMode::EXTERNAL && (turnSignalMessage_.turn_signal.data == TurnSignal::LEFT || turnSignalMessage_.turn_signal.data == TurnSignal::RIGHT)) {
        LED_BLINK(ledBlinkState_, ledBlinkLastTime_, this, joy_, joyFeedbackPublisher_, 0xFF, 0xFF, 0x00, 100ms);
    } else if (gateCommandMessage_.data == GateMode::EXTERNAL) {
        joy_.publishLEDMessage(joyFeedbackPublisher_, 0xFF, 0xFF, 0x00);
    } else if (stateMessage_.state == AutowareState::DRIVING) {
        LED_BLINK(ledBlinkState_, ledBlinkLastTime_, this, joy_, joyFeedbackPublisher_, 0xFF, 0x00, 0xFF, 100ms);
    } else if (stateMessage_.state == AutowareState::FINALIZING) {
        joy_.publishLEDMessage(joyFeedbackPublisher_, 0xFF, 0x00, 0xFF);
    } else if (stateMessage_.state == AutowareState::WAITING_FOR_ENGAGE) {
        joy_.publishLEDMessage(joyFeedbackPublisher_, 0x00, 0xFF, 0x00);
    } else if (gateCommandMessage_.data == GateMode::AUTO) {
        LED_BLINK(ledBlinkState_, ledBlinkLastTime_, this, joy_, joyFeedbackPublisher_, 0x00, 0xFF, 0x00, 100ms);
    }
}

void BaiduJoyControllerNode::toControlCommandMessage()
{
    tier4_external_api_msgs::msg::ControlCommandStamped cmd;
    cmd.stamp = this->get_clock()->now();
    cmd.control.steering_angle = steeringMaxAngle_ * joy_.LStickLeftRight();
    cmd.control.brake = joy_.LTrigger();
    if (gearMessage_.gear_shift.data == tier4_external_api_msgs::msg::GearShift::DRIVE) {
        cmd.control.throttle = std::min(forwardRatio_ * joy_.RTrigger(), 1.0f);
    } else if (gearMessage_.gear_shift.data == tier4_external_api_msgs::msg::GearShift::REVERSE) {
        cmd.control.throttle = std::min(backwardRatio_ * joy_.RTrigger(), 1.0f);
    } else {
        /* 只有前进和后退挡才给速度,其他的直接0 */
        cmd.control.throttle = 0;
    }
    controlCommandPublisher_->publish(cmd);
}

void BaiduJoyControllerNode::toGearShiftMessage()
{
    gearMessage_.stamp = this->get_clock()->now();
    /* 转换档位 */
    if (joy_.CursorLeft()) {
        gearMessage_.gear_shift.data = tier4_external_api_msgs::msg::GearShift::NEUTRAL;
    } else if (joy_.CursorRight()) {
        gearMessage_.gear_shift.data = tier4_external_api_msgs::msg::GearShift::PARKING;
    } else if (joy_.CursorTop()) {
        gearMessage_.gear_shift.data = tier4_external_api_msgs::msg::GearShift::DRIVE;
    } else if (joy_.CursorBottom()) {
        gearMessage_.gear_shift.data = tier4_external_api_msgs::msg::GearShift::REVERSE;
    }
    gearShiftPublisher_->publish(gearMessage_);
}

void BaiduJoyControllerNode::toTurnSignalMessage()
{
    turnSignalMessage_.stamp = this->get_clock()->now();
    /* 转向灯 */
    if (CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, L1) || CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, R1)) {
        if (joy_.L1()) {
            if (turnSignalMessage_.turn_signal.data == tier4_external_api_msgs::msg::TurnSignal::LEFT) {
                turnSignalMessage_.turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::NONE;
            } else {
                turnSignalMessage_.turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::LEFT;
            }
        } else if (joy_.R1()) {
            if (turnSignalMessage_.turn_signal.data == tier4_external_api_msgs::msg::TurnSignal::RIGHT) {
                turnSignalMessage_.turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::NONE;
            } else {
                turnSignalMessage_.turn_signal.data = tier4_external_api_msgs::msg::TurnSignal::RIGHT;
            }
        }
    }
    turnSignalPublisher_->publish(turnSignalMessage_);
}

void BaiduJoyControllerNode::toGateModeMessage()
{
    /* 摇杆模式切换 */
    if (CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, Cross)) {
        if (joy_.Cross()) {
            if (gateCommandMessage_.data == tier4_control_msgs::msg::GateMode::AUTO) {
                gateCommandMessage_.data = tier4_control_msgs::msg::GateMode::EXTERNAL;
            } else {
                gateCommandMessage_.data = tier4_control_msgs::msg::GateMode::AUTO;
            }
        }
    }
    gateModeCommandPublisher_->publish(gateCommandMessage_);
}

void BaiduJoyControllerNode::toHeartbeatMessage()
{
    tier4_external_api_msgs::msg::Heartbeat msg;
    msg.stamp = this->get_clock()->now();
    heartbeatPublisher_->publish(msg);
}

void BaiduJoyControllerNode::toVehicleEngageMessage()
{
    vehicleEngageMessage_.stamp = this->get_clock()->now();
    if (joy_.PS()) {
        vehicleEngageMessage_.engage = true;
    }
    vehicleEngagePublisher_->publish(vehicleEngageMessage_);
}

void BaiduJoyControllerNode::callClientEngageService()
{
    clientEngageService_->async_send_request(clientEngageServiceRequest_, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
        auto response = result.get();
        if (tier4_api_utils::is_success(response->status)) {
            RCLCPP_INFO(get_logger(), "Set Client Engage succeeded");
        } else {
            RCLCPP_WARN(get_logger(), "Set Client Engage failed: %s", response->status.message.c_str());
        }
    });
}

void BaiduJoyControllerNode::callEmergencyService()
{
    emergencyStopService_->async_send_request(emergencyServiceRequest_, [this](rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture result) {
        auto response = result.get();
        if (tier4_api_utils::is_success(response->status)) {
            RCLCPP_INFO(get_logger(), "Set Emergency succeeded");
        } else {
            RCLCPP_WARN(get_logger(), "Set Emergency failed: %s", response->status.message.c_str());
        }
    });
}

void BaiduJoyControllerNode::callAutowareEngageService()
{
    autowareEngageService_->async_send_request(autowareEngageServiceRequest_, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
        auto response = result.get();
        if (tier4_api_utils::is_success(response->status)) {
            RCLCPP_INFO(get_logger(), "Set Autoware Engage succeeded");
        } else {
            RCLCPP_WARN(get_logger(), "Set Autoware Engage failed: %s", response->status.message.c_str());
        }
    });
}

void BaiduJoyControllerNode::toEmergencyService()
{
    /* 摇杆离线直接触发急停 */
    if (lastReceivedValidJoy_ + joyTimeout_ < this->get_clock()->now()) {
        if (!emergencyServiceRequest_->emergency) {
            emergencyServiceRequest_->emergency = true;
            callEmergencyService();
        }
        return;
    }
    if (CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, Square) || CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, Circle)) {
        if (joy_.Square()) {
            emergencyServiceRequest_->emergency = true;
        } else if (joy_.Circle()) {
            emergencyServiceRequest_->emergency = false;
        }
        callEmergencyService();
    }
}

void BaiduJoyControllerNode::toAutowareEngageService()
{
    if (CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, Triangle)) {
        if (joy_.Triangle()) {
            autowareEngageServiceRequest_->engage = !autowareEngageServiceRequest_->engage;
        }
        callAutowareEngageService();
    }
}

void BaiduJoyControllerNode::toClientEngageService()
{
    if (CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy_, lastJoy_, PS)) {
        clientEngageServiceRequest_->engage = true;
        callClientEngageService();
    }
}

}  // namespace baidu_joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(baidu_joy_controller::BaiduJoyControllerNode)