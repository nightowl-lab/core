#include "nightowl_remote_controller/webrtc.hpp"
#include "nightowl_remote_controller/tool.hpp"
#include <cstdlib>

namespace nightowl_remote_controller
{

WebRTC::WebRTC(rclcpp::Node & node, double fps, std::vector<std::string> iceServers, std::string signalingURL, std::string signalingUsername, std::string signalingPassword)
: node_(node), iceServers_(iceServers), signalingURL_(signalingURL), fps_(fps), signalingPassword_(signalingPassword), signalingUsername_(signalingUsername)
{
    initWebsocket();
    initRTC();
}

void WebRTC::initWebsocket()
{
    std::scoped_lock lock(threadLock_);
    websocket_.onOpen([&]() {
        RCLCPP_INFO(node_.get_logger(), "Signaling server connected, try to login");
        nlohmann::json data;
        data["username"] = signalingUsername_;
        data["password"] = signalingPassword_;
        data["role"] = "VEHICLE";
        loginCallID_ = sendWebSocketMessage("server", -1, "login", data);
    });

    websocket_.onClosed([&]() { RCLCPP_INFO(node_.get_logger(), "Signaling server disconnected"); });

    websocket_.onError([&](const std::string & error) { RCLCPP_ERROR(node_.get_logger(), "Signaling server error: %s", error.c_str()); });

    websocket_.onMessage([&](rtc::message_variant data) {
        if (!std::holds_alternative<std::string>(data)) return;

        nlohmann::json message = nlohmann::json::parse(std::get<std::string>(data));
        onWebsocketMessage(message);
    });

    websocket_.open(signalingURL_);

    RCLCPP_WARN(node_.get_logger(), "Waiting for signaling to be connected...");
    while (!websocket_.isOpen()) {
        rclcpp::sleep_for(100ms);
    }
}

void WebRTC::printSDP(std::string sdp)
{
    RCLCPP_INFO(node_.get_logger(), "------Begin SDP------");
    size_t last = 0;
    size_t next = 0;
    while ((next = sdp.find("\r\n", last)) != std::string::npos) {
        RCLCPP_INFO(node_.get_logger(), sdp.substr(last, next - last).c_str());
        last = next + 2;
    }
    RCLCPP_INFO(node_.get_logger(), "------End SDP------");
}

void WebRTC::initRTC()
{
    std::scoped_lock lock(threadLock_);
    /* 写入ICE服务器 */
    rtc::Configuration config;
    for (auto iter = iceServers_.begin(); iter != iceServers_.end(); iter++) {
        config.iceServers.push_back(rtc::IceServer(*iter));
    }
    config.disableAutoNegotiation = true;

    peer_.reset(new rtc::PeerConnection(config));
    peer_->onStateChange([&](rtc::PeerConnection::State state) { RCLCPP_INFO(node_.get_logger(), "WebRTC State changed: %d", (int)state); });
    peer_->onGatheringStateChange([&](rtc::PeerConnection::GatheringState state) {
        RCLCPP_INFO(node_.get_logger(), "WebRTC ICE Gathering State changed: %d", (int)state);
        if (state == rtc::PeerConnection::GatheringState::Complete) {
            std::scoped_lock lock(threadLock_);
            auto description = peer_->localDescription();
            offer_ = std::string(description.value());
            if (receivedOfferCallback_ != NULL) {
                receivedOfferCallback_(offer_);
            }
            RCLCPP_INFO(node_.get_logger(), "WebRTC Got offer!");
            printSDP(offer_);
        }
    });

    /* 图传信道 */
    auto video = rtc::Description::Video("fpv", rtc::Description::Direction::SendOnly);
    video.addH264Codec(102);
    video.addSSRC(1, "fpv", "stream0", "fpv");
    videoTrack_ = peer_->addTrack(video);
    auto rtpConfig = std::make_shared<rtc::RtpPacketizationConfig>(1, "fpv", 102, rtc::H264RtpPacketizer::defaultClockRate);
    auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(rtc::H264RtpPacketizer::Separator::StartSequence, rtpConfig);
    auto h264Handler = std::make_shared<rtc::H264PacketizationHandler>(packetizer);
    rtcpReporter_.reset(new rtc::RtcpSrReporter(rtpConfig));
    h264Handler->addToChain(rtcpReporter_);
    h264Handler->addToChain(std::make_shared<rtc::RtcpNackResponder>());
    videoTrack_->setMediaHandler(h264Handler);
    videoTrack_->onOpen([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC MediaChannel opened");
        std::scoped_lock lock(threadLock_);
        shouldSendMediaPacket_ = true;
        shouldResetTime_ = true;
    });
    videoTrack_->onClosed([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC MediaChannel closed");
        std::scoped_lock lock(threadLock_);
        shouldSendMediaPacket_ = false;
    });

    /* 数传信道 */
    reliableChannel_ = std::move(peer_->createDataChannel("reliable"));
    reliableChannel_->onOpen([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC Reliable DataChannel opened");
        shouldSendReliablePacket_ = true;
    });
    reliableChannel_->onClosed([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC Reliable DataChannel closed");
        shouldSendReliablePacket_ = false;
    });
    reliableChannel_->onMessage(std::bind(&WebRTC::onRPCCall, this, std::placeholders::_1), [&](std::string data) {
        RCLCPP_WARN(node_.get_logger(), "WebRTC Reliable DataChannel doesn't support string data!");
    });

    unreliableChannel_ = std::move(peer_->createDataChannel("unreliable"));
    unreliableChannel_->onOpen([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC Unreliable DataChannel opened");
        shouldSendUnreliablePacket_ = true;
    });
    unreliableChannel_->onClosed([&]() {
        RCLCPP_INFO(node_.get_logger(), "WebRTC Unreliable DataChannel closed");
        shouldSendUnreliablePacket_ = false;
    });

    peer_->setLocalDescription();
}

void WebRTC::sendOffer(std::string clientID, std::string offer)
{
    nlohmann::json sendMessage;
    sendMessage[0] = offer_;
    setOfferCallID_ = sendWebSocketMessage(connectedSignalingClient_, -1, "offer", sendMessage);
    receivedOfferCallback_ = NULL;
    RCLCPP_INFO(node_.get_logger(), "Sent WebRTC Offer!");
}

int WebRTC::sendWebSocketMessage(std::string to, int callID, std::string type, nlohmann::json data)
{
    nlohmann::json sendMessage;
    callID = callID == -1 ? rand() : callID;
    sendMessage["to"] = to;
    sendMessage["callID"] = callID;
    sendMessage["type"] = type;
    sendMessage["data"] = data;
    websocket_.send(sendMessage.dump());
    return callID;
}

void WebRTC::onWebsocketMessage(nlohmann::json & message)
{
    RCLCPP_INFO(node_.get_logger(), "%s", message.dump().c_str());
    std::scoped_lock lock(threadLock_);
    auto it = message.find("from");
    if (it == message.end()) {
        return;
    }
    std::string from = it->get<std::string>();

    it = message.find("callID");
    if (it == message.end()) {
        return;
    }
    int callID = it->get<int>();

    it = message.find("type");
    if (it == message.end()) {
        return;
    }
    std::string type = it->get<std::string>();

    auto dataIter = message.find("data");
    if (dataIter == message.end()) {
        return;
    }

    if (type == "request") {
        nlohmann::json sendData = {{"code", 0}};
        sendWebSocketMessage(from, callID, "response", sendData);
        receivedOfferCallback_ = std::bind(&WebRTC::sendOffer, this, from, std::placeholders::_1);
        if (offer_ == "") {
            RCLCPP_WARN(node_.get_logger(), "Waiting for WebRTC Offer ready");
            return;
        }
        if (connectedSignalingClient_ != "" && connectedSignalingClient_ != from) {
            RCLCPP_WARN(node_.get_logger(), "A WebRTC connection is already exists, try to close it and reinitalize.");
            peer_->close();
            initRTC();
            return;
        }
        connectedSignalingClient_ = from;
        receivedOfferCallback_(offer_);
    } else if (type == "response") {
        it = dataIter->find("code");
        if (it == dataIter->end()) {
            return;
        }
        int code = it->get<int>();

        if (callID == loginCallID_) {
            loginCallID_ = -1;
            if (code != 0) {
                RCLCPP_FATAL(node_.get_logger(), "Failed to login to signaling server, code: %d", code);
            } else {
                RCLCPP_INFO(node_.get_logger(), "Login to signaling server successful!");
            }
        } else if (callID == setOfferCallID_) {
            setOfferCallID_ = -1;
            it = dataIter->find("sdp");
            if (it == dataIter->end()) {
                return;
            }
            std::string sdp = it->get<std::string>();
            RCLCPP_INFO(node_.get_logger(), "Received WebRTC Answer from Client: %s", from.c_str());
            printSDP(sdp);
            auto description = rtc::Description(sdp, "answer");
            peer_->setRemoteDescription(description);
        }
    }
}

void WebRTC::onH264PacketReceived(uint8_t * packet, size_t length)
{
    if (!shouldSendMediaPacket_) return;
    double shouldResetTime = false;
    {
        std::scoped_lock lock(threadLock_);
        shouldResetTime = shouldResetTime_;
    }
    if (shouldResetTime_) {
        /* 每次重新建立流的时候需要重制时间戳 */
        auto usTimestamp = node_.get_clock()->now().nanoseconds() / 1000.0f;
        rtcpReporter_->rtpConfig->setStartTime(usTimestamp, rtc::RtpPacketizationConfig::EpochStart::T1970);
        rtcpReporter_->rtpConfig->timestamp = usTimestamp;
        rtcpReporter_->startRecording();
        shouldResetTime_ = false;
    } else {
        rtcpReporter_->rtpConfig->timestamp += 1e6 * 100 / (uint32_t)(fps_ * 100.0f);
    }
    /* 发送RTCP报告 */
    auto reportElapsedTimestamp = rtcpReporter_->rtpConfig->timestamp - rtcpReporter_->previousReportedTimestamp;
    if (rtcpReporter_->rtpConfig->timestampToSeconds(reportElapsedTimestamp) > 1) {
        rtcpReporter_->setNeedsToReport();
    }

    try {
        videoTrack_->send((std::byte *)packet, length);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node_.get_logger(), "Unable to send video packet, size: %s", e.what());
    }
}

void WebRTC::sendReportBinaryMessage(std::vector<std::byte> data)
{
    if (!shouldSendUnreliablePacket_) return;
    assert(data.size() <= 14);
    std::vector<std::byte> byteData_;
    byteData_.push_back(std::byte(WEBRTC_REPORT_DATA_SOF));
    byteData_.insert(byteData_.end(), data.begin(), data.end());
    byteData_.resize(15, std::byte(0));
    byteData_.push_back(static_cast<std::byte>(Tool::crc8(byteData_)));
    unreliableChannel_->send(byteData_);
}

void WebRTC::sendVehicleReport(
  double speed, double steeringAngle, AutowareState autowareState, GearState gearState, double batteryPercentage, bool vehicleStopMode, bool emergencyStop, bool leftTurnIndicator,
  bool rightTurnIndicator)
{
    std::vector<std::byte> data;

    short sendSpeed = speed * 100.0f;
    data.push_back(std::byte(sendSpeed >> 8));
    data.push_back(std::byte(sendSpeed & 0xFF));

    short sendAngle = steeringAngle * 100.0f;
    data.push_back(std::byte(sendAngle >> 8));
    data.push_back(std::byte(sendAngle & 0xFF));

    data.push_back(std::byte(autowareState));

    data.push_back(std::byte(gearState));

    uint8_t sendPercentage = batteryPercentage * 100.0f;
    data.push_back(std::byte(sendPercentage));

    uint8_t sendBinaryData = vehicleStopMode | (emergencyStop << 1) | (leftTurnIndicator << 2) | (rightTurnIndicator << 3);
    data.push_back(std::byte(sendBinaryData));

    sendReportBinaryMessage(data);
}

void WebRTC::onRPCCall(std::vector<std::byte> data)
{
    auto tmp = std::vector<std::byte>(data.begin(), data.end() - 1);
    if (data.size() < 5 || data[0] != std::byte(WEBRTC_RPC_CALL_DATA_SOF) || (uint8_t)data[data.size() - 1] != Tool::crc8(std::vector<std::byte>(data.begin(), data.end() - 1))) {
        RCLCPP_WARN(node_.get_logger(), "Invalid RPC Call Message Received!");
        return;
    }
    uint16_t callID = (uint8_t)data[1] << 8 | (uint8_t)data[2];
    RPCType callType = (RPCType)data[3];
    if (callType == RPCType::PING) {
        sendRPCReturn(callID, 0, std::vector<std::byte>());
    }
}

void WebRTC::sendRPCReturn(uint16_t callID, int8_t code, std::vector<std::byte> data)
{
    if (!shouldSendReliablePacket_) return;
    std::vector<std::byte> byteData_;
    byteData_.push_back(std::byte(WEBRTC_RPC_RETURN_DATA_SOF));
    byteData_.push_back(std::byte(callID >> 8));
    byteData_.push_back(std::byte(callID & 0xFF));
    byteData_.push_back(std::byte(code));
    byteData_.insert(byteData_.end(), data.begin(), data.end());
    byteData_.push_back(static_cast<std::byte>(Tool::crc8(byteData_)));
    reliableChannel_->send(byteData_);
}

}  // namespace nightowl_remote_controller
