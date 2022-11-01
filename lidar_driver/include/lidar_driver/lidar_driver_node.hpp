#ifndef _LIDAR_DRIVER_H
#define _LIDAR_DRIVER_H

#include <chrono>
#include <thread>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/time_reference.hpp>

#include "lidar_driver/lidar_input.hpp"

namespace lidar_driver
{
    class LidarDriver : public rclcpp::Node
    {
    public:
        explicit LidarDriver(const rclcpp::NodeOptions& nodeOptions);

        ~LidarDriver();

        bool msopPoll(void);
        void difopPoll(void);
    private:

        void skipNumCallback(const std_msgs::msg::Int32::ConstPtr& skip_num);

        struct
        {
            std::string frame_id;
            std::string model;
            int npackets;
            double rpm;
            double time_offset;
            int cut_angle;
            int return_mode;
        }config_;

        int msop_udp_port;
        int difop_udp_port;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<lidar_msgs::msg::LidarScanUnified>::SharedPtr msopPublisher_;
        rclcpp::Publisher<lidar_msgs::msg::LidarPacket>::SharedPtr      difopPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr   synchronizationPublisher_;

        /* difop数据包的解析线程 */
        std::shared_ptr<std::thread> difop_thread_;

        /* 用于时间同步 */
        bool time_synchronization_;
        unsigned char packetTimeStamp[10];
        uint64_t pointcloudTimeStamp;
        uint64_t GPSStableTS;
        uint64_t GPSCountingTS;
        uint64_t last_FPGA_ts;
        uint64_t GPS_ts;
        int cnt_gps_ts;
        rclcpp::Time timeStamp;
        uint64_t usec_start;
    };
}

#endif