#include "nightowl_remote_controller/nightowl_remote_controller_node.hpp"

#include "nightowl_remote_controller/tool.hpp"

namespace nightowl_remote_controller
{

NightOwlRemoteControllerNode::NightOwlRemoteControllerNode(const rclcpp::NodeOptions & nodeOptions) : Node("nightowl_remote_controller", nodeOptions)
{
    /* 读取配置 */
    signalingServerURL_ = this->declare_parameter("signaling_server_url", "ws://127.0.0.1:8000/server");
    fps_ = this->declare_parameter("fps", 12.5);
    bitrate_ = this->declare_parameter("bitrate", 1000000);
    iceServers_ = this->declare_parameter("ice_servers", std::vector<std::string>({}));
    signalingServerUsername_ = this->declare_parameter("signaling_server_username", "baidu");
    signalingServerPassword_ = this->declare_parameter("signaling_server_password", "baidu");
    webrtc_.reset(new WebRTC(*this, fps_, iceServers_, signalingServerURL_, signalingServerUsername_, signalingServerPassword_));
    /* 订阅话题 */
    imageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("input/image", 10, std::bind(&NightOwlRemoteControllerNode::imageCallback, this, std::placeholders::_1));
    subscribeReport("input/gear_report", gearReportSubscriber_, lastGearReport_);
    subscribeReport("input/control_mode_report", controlModeReportSubscriber_, lastControlModeReport_);
    subscribeReport("input/steering_report", steeringReportSubscriber_, lastSteeringReport_);
    subscribeReport("input/velocity_report", velocityReportSubscriber_, lastVelocityReport_);
    subscribeReport("input/turn_indicators_report", turnIndicatorsSubscriber_, lastTurnIndicatorsReport_);
    subscribeReport("input/emergency", emergencySubscriber_, lastEmergency_);
    subscribeReport("input/gate_mode", gateModeSubscriber_, lastGateMode_);
    subscribeReport("input/battery_flag_feedback", batteryFlagFeedbackSubscriber_, lastBatteryFlagFeedback_);
    subscribeReport("input/autoware_state", autowareStateSubscriber_, lastAutowareState_);

    /* 初始化主循环 */
    timer_ = rclcpp::create_timer(this, get_clock(), 5ms, std::bind(&NightOwlRemoteControllerNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "nightowl_remote_controller_node started.");
}

template <typename T>
void NightOwlRemoteControllerNode::subscribeReport(std::string topicName, typename rclcpp::Subscription<T>::SharedPtr & subscription, T & lastMessageContainer)
{
    waitingForReportTopic_.push_back(topicName);
    subscription = this->create_subscription<T>(topicName, 10, [&, closureTopicName = topicName](const typename T::SharedPtr msg) {
        if (!allReportTopicReady_) {
            waitingForReportTopic_.erase(std::remove(waitingForReportTopic_.begin(), waitingForReportTopic_.end(), closureTopicName), waitingForReportTopic_.end());
            if (waitingForReportTopic_.size() == 0) {
                allReportTopicReady_ = true;
            }
        }
        lastMessageContainer = *msg;
    });
}

void NightOwlRemoteControllerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
    if (!allReportTopicReady_) return;
    if (!isInitalized_) {
        encoder_.reset(new H264Encoder(*this, image->width, image->height, bitrate_, fps_));
        isInitalized_ = true;
    }
    encoder_->sendImage(image->data);
}

void NightOwlRemoteControllerNode::sendVehicleReport()
{
    /* 转换Autoware状态 */
    AutowareState state = static_cast<AutowareState>(lastAutowareState_.state);
    if (lastGateMode_.data == tier4_control_msgs::msg::GateMode::EXTERNAL) {
        state = AutowareState::MANUAL;
    }
    /* 转换档位信息 */
    GearState gear;
    if (lastGearReport_.report == autoware_auto_vehicle_msgs::msg::GearReport::DRIVE) {
        gear = GearState::D;
    } else if (lastGearReport_.report == autoware_auto_vehicle_msgs::msg::GearReport::REVERSE) {
        gear = GearState::R;
    } else if (lastGearReport_.report == autoware_auto_vehicle_msgs::msg::GearReport::PARK) {
        gear = GearState::P;
    } else {
        gear = GearState::N;
    }
    /* 转换二值类属性 */
    bool vehicleStopMode =
      lastControlModeReport_.mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::NOT_READY || lastControlModeReport_.mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
    bool leftTurnIndicator = lastTurnIndicatorsReport_.report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    bool rightTurnIndicator = lastTurnIndicatorsReport_.report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    double steeringAngle = lastSteeringReport_.steering_tire_angle * TOOL_RAD_TO_DEG;
    webrtc_->sendVehicleReport(
      lastVelocityReport_.longitudinal_velocity, steeringAngle, state, gear, lastBatteryFlagFeedback_.percent, vehicleStopMode, lastEmergency_.emergency, leftTurnIndicator, rightTurnIndicator);
}

void NightOwlRemoteControllerNode::timerCallback()
{
    if (!allReportTopicReady_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for message ready...");
        return;
    }
    if (isInitalized_) {
        auto packet = encoder_->receiveImage();
        if (packet.first) {
            sendVehicleReport();
            webrtc_->onH264PacketReceived(packet.first, packet.second);
        }
    }
}

}  // namespace nightowl_remote_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nightowl_remote_controller::NightOwlRemoteControllerNode)