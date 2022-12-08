#ifndef NIGHTOWL_REMOTE_CONTROLLER__WEBSOCKET_H__
#define NIGHTOWL_REMOTE_CONTROLLER__WEBSOCKET_H__

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtc/rtc.hpp>

#define WEBRTC_REPORT_DATA_SOF 0xAA
#define WEBRTC_JOYSTICK_DATA_SOF 0xBB
#define WEBRTC_RPC_CALL_DATA_SOF 0xCC
#define WEBRTC_RPC_RETURN_DATA_SOF 0xDD

using namespace std::chrono_literals;

namespace nightowl_remote_controller
{

enum class AutowareState {
    INITIALIZING = 1u,
    WAITING_FOR_ROUTE = 2u, 
    PLANNING = 3u,
    WAITING_FOR_ENGAGE = 4u,
    DRIVING = 5u,
    ARRIVED_GOAL = 6u,
    FINALIZING = 7u,
    MANUAL = 100u
};


enum class GearState {
    N = 1u,
    P = 2u,
    D = 3u,
    R = 4u
};

enum class RPCType {
    PING = 1u
};

class WebRTC
{
  public:
    /**
     * Construct a new WebRTC object
     *
     * @param node ROS实例
     * @param fps 帧率
     * @param iceServers 候选ICE服务器列表
     * @param signalingURL WebSocket信令服务器地址
     * @param signalingUsername WebSocket信令服务器用户名
     * @param signalingPassword WebSocket信令服务器密码
     */
    WebRTC(rclcpp::Node & node, double fps, std::vector<std::string> iceServers, std::string signalingURL, std::string signalingUsername, std::string signalingPassword);

    /**
     * 接收到H264Packet回调
     *
     * @param packet H264包指针
     * @param length 长度
     */
    void onH264PacketReceived(uint8_t * packet, size_t length);

    /**
     * 发送车辆报告
     * 
     * @param speed 速度,单位km/h
     * @param steeringAngle 转向角度,单位度
     * @param autowareState Autoware状态
     * @param gearState 档位状态
     * @param batteryPercentage 电池电量百分比
     * @param vehicleStopMode 是否处于底盘急停状态
     * @param emergencyStop 是否处于手动急停状态
     * @param leftTurnIndicator 左转向灯状态
     * @param rightTurnIndicator 右转向灯状态
     */
    void sendVehicleReport(double speed, double steeringAngle, AutowareState autowareState, GearState gearState, double batteryPercentage, bool vehicleStopMode, bool emergencyStop, bool leftTurnIndicator, bool rightTurnIndicator);

  private:
    rclcpp::Node & node_;                                                 /* ROS节点 */
    std::vector<std::string> iceServers_;                                 /* 候选ICE服务器列表 */
    std::string signalingURL_;                                            /* WebSocket信令服务器地址 */
    rtc::WebSocket websocket_;                                            /* WebSocket信令实例 */
    std::string signalingUsername_;                                       /* WebSocket信令登录用户名 */
    std::string signalingPassword_;                                       /* WebSocket信令登录密码 */
    std::string offer_ = "";                                              /* 已经接受到的WebRTC Offer */
    std::unique_ptr<rtc::PeerConnection> peer_;                           /* WebRTC连接实例 */
    std::shared_ptr<rtc::DataChannel> unreliableChannel_;                 /* WebRTC不可靠传输类数据通道 */
    std::shared_ptr<rtc::DataChannel> reliableChannel_;                   /* WebRTC可靠类数据通道 */
    std::shared_ptr<rtc::RtcpSrReporter> rtcpReporter_;                   /* WebRTC媒体质量报告 */
    std::shared_ptr<rtc::Track> videoTrack_;                              /* WebRTC视频通道。*/
    bool shouldResetTime_ = false;                                        /* 是否在下一次接受到ROS图像数据时重置RTC时间戳 */
    bool shouldSendReliablePacket_ = false;                               /* 是否允许发送可靠类数据包 */
    bool shouldSendUnreliablePacket_ = false;                             /* 是否允许发送不可靠类数据包 */
    bool shouldSendMediaPacket_ = false;                                  /* 是否允许发送视频数据包 */
    std::recursive_mutex threadLock_;                                     /* 线程安全锁 */
    double fps_;                                                          /* 配置帧率 */
    std::string connectedSignalingClient_;                                /* 当前已经建立信令连接的客户端ID */
    int loginCallID_;                                                     /* 登录信令的调用ID */
    int setOfferCallID_;                                                  /* 设置Offer信令的调用ID */
    std::function<void(std::string offer)> receivedOfferCallback_ = NULL; /* 成功接受到RTC Offer时回调*/

    /**
     * 打印SDP描述
     * 
     * @param sdp SDP字符串
     */
    void printSDP(std::string sdp);

    /**
     * 发送RPC调用返回值
     * 
     * @param callID 调用ID
     * @param code 返回值
     * @param data 返回数据
     */
    void sendRPCReturn(uint16_t callID, int8_t code, std::vector<std::byte> data);

    /**
     * RPC调用消息回调
     * 
     * @param data RPC数据包
     */
    void onRPCCall(std::vector<std::byte> data);

    /**
     * 发送车辆报告类型二进制数据
     * 
     * @param data 车辆报告数据
     */
    void sendReportBinaryMessage(std::vector<std::byte> data);

    /**
     * WebSocket消息回调
     *
     * @param message
     */
    void onWebsocketMessage(nlohmann::json & message);

    /**
     * 通过WebSocket发送offer
     *
     * @param clientID 客户端ID
     * @param offer WebRTC Offer
     */
    void sendOffer(std::string clientID, std::string offer);

    /**
     * 发送WebSocket信息
     * 
     * @param to 目标
     * @param callID 调用ID,若为-1则自动生成
     * @param type 调用类型
     * @param data 数据
     * @return 调用ID
     */
    int sendWebSocketMessage(std::string to, int callID, std::string type, nlohmann::json data);

    /**
     * 初始化WebSocket
     *
     */
    void initWebsocket();

    /**
     * 初始化WebRTC Peer Connection
     *
     */
    void initRTC();
};

}  // namespace nightowl_remote_controller

#endif