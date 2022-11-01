#ifndef __LIDAR_INPUT_H
#define __LIDAR_INPUT_H

#include <pcap.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_msgs/msg/lidar_packet.hpp"
#include "lidar_msgs/msg/lidar_scan_unified.hpp"

namespace lidar_driver
{
    static const uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lidar default data port on PC
    static const uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lidar default difop data port on PC

    class Input
    {
    public:
        Input(rclcpp::Node::SharedPtr node_ptr, uint16_t port);

        virtual ~Input();

        virtual int getPacket(lidar_msgs::msg::LidarPacket* pkt, const double time_offset) = 0;

        int getRpm(void);
        int getReturnMode(void);
        bool getUpdateFlag(void);
        void clearUpdateFlag(void);

    protected:
        rclcpp::Node::SharedPtr node_ptr_;
        uint16_t port_;
        std::string device_ip_str_;
        int current_rpm_;
        int return_mode_;
        bool npkt_update_flag_;
    };

    class InputSocket : public Input
    {
    public:
        InputSocket(rclcpp::Node::SharedPtr node_ptr, uint16_t port = MSOP_DATA_PORT_NUMBER);

        virtual ~InputSocket();

        virtual int getPacket(lidar_msgs::msg::LidarPacket* pkt, const double time_offset);

    private:
        int socket_fd_;
        in_addr device_ip_;
    };

    // class InputPCAP : public Input
    // {
    // public:
    //     InputPCAP(rclcpp::Node::SharedPtr node_ptr, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
    //         std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

    //     virtual ~InputPCAP();

    //     virtual int getPacket(lidar_msgs::msg::LidarPacket* pkt, const double time_offset);

    // private:
    //     rclcpp::Rate packet_rate_;
    //     std::string filename_;
    //     pcap_t* pcap_;
    //     bpf_program pcap_packet_filter_;
    //     char errbuf_[PCAP_ERRBUF_SIZE];
    //     bool empty_;
    //     bool read_once_;
    //     bool read_fast_;
    //     double repeat_delay_;
    // };
}

#endif