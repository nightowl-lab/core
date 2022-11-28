#ifndef _LIDAR_DECODER_H
#define _LIDAR_DECODER_H

#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include "lidar_decoder/pointcloudXYZIRADT.h"
#include "lidar_decoder/lidar_rawdata.hpp"
#include <queue>

namespace lidar_decoder
{
    class LidarDecoder : public rclcpp::Node
    {
    public:
        explicit LidarDecoder(const rclcpp::NodeOptions& nodeOptions);
    
    private:
        void processScan(const lidar_msgs::msg::LidarScanUnified& scanMsg);
        void timeSync(const sensor_msgs::msg::TimeReference& time_msg);

        bool time_synchronization_;
        rclcpp::Time global_time;
        double last_time;
        std::vector<int> indices;
        std::shared_ptr<lidar_rawdata::RawData> data_;
        int64_t lastSec;
        int64_t lastNanosec;
        rclcpp::Subscription<lidar_msgs::msg::LidarScanUnified>::SharedPtr packetSubscriber_;
        rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr synchronizationSubscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudExPublisher_;

    };
}

#endif