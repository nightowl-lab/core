#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include "lidar_decoder/lidar_decoder_node.hpp"
#include "lidar_decoder/lidar_rawdata.hpp"

static void abort_handler(int sig)
{
    abort();
}

namespace lidar_decoder
{
    std::string model;

    LidarDecoder::LidarDecoder(const rclcpp::NodeOptions& nodeOptions)
        : Node("lidar_decoder", nodeOptions), data_(new lidar_rawdata::RawData((rclcpp::Node::SharedPtr)this))
    {
        signal(SIGINT, abort_handler);
        this->data_->loadConfigFile();

        if(!this->has_parameter("model"))
        {
            model = this->declare_parameter("model", std::string("LSC16"));
        }

        this->pointCloudExPublisher_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/point_cloud", rclcpp::SensorDataQoS());

        this->packetSubscriber_ = this->create_subscription<lidar_msgs::msg::LidarScanUnified>("input/msop_packet", 10, std::bind(&LidarDecoder::processScan, this, std::placeholders::_1));

        this->time_synchronization_ = this->declare_parameter("time_synchronization", false);

        if(this->time_synchronization_)
        {
            this->synchronizationSubscriber_ = this->create_subscription<sensor_msgs::msg::TimeReference>("input/sync_header", 10, std::bind(&LidarDecoder::timeSync, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(), "\033[1;32m<--lidar_decoder_node started.-->\033[0m");
    }

    void LidarDecoder::timeSync(const sensor_msgs::msg::TimeReference& time_msg)
    {
        this->global_time = time_msg.header.stamp;
    }

    void LidarDecoder::processScan(const lidar_msgs::msg::LidarScanUnified& scanMsg)
    {
        pcl::PointCloud<lslidar_pointcloud::PointXYZIRADT>::Ptr outPoints(new pcl::PointCloud<lslidar_pointcloud::PointXYZIRADT>);

        outPoints->header.frame_id = scanMsg.header.frame_id;

        outPoints->clear();
        outPoints->height = 16;
        outPoints->width = 24 * (int) scanMsg.packets.size();

        outPoints->is_dense = false;
        outPoints->resize(outPoints->height * outPoints->width);

        // process each packet provided by the driver
        this->data_->block_num = 0;

        double deltaTime = 0;
        uint32_t firstPacketEndTime = 0;
        if (scanMsg.packets.size() > 1) {
            memcpy(&firstPacketEndTime, &scanMsg.packets[0].data[1200], 4);
            deltaTime = ((double)firstPacketEndTime - lidar_rawdata::LSC16_BLOCK_TDURATION * 11) - 31 * lidar_rawdata::LSC16_DSR_TOFFSET;
        }
        for(size_t i = 0; i < scanMsg.packets.size(); i++)
        {
            this->data_->unpack(scanMsg.packets[i], outPoints, deltaTime);
        }

        sensor_msgs::msg::PointCloud2 outMsg;

        // removeNaNFromPointCloud
        std::size_t j = 0;
        for (std::size_t i = 0; i < outPoints->points.size(); ++i)
        {
          if (!std::isfinite (outPoints->points[i].x) ||
              !std::isfinite (outPoints->points[i].y) ||
              !std::isfinite (outPoints->points[i].z))
            continue;
          outPoints->points[j] = outPoints->points[i];
          j++;
        }

        if (j != outPoints->points.size ())
        {
          // Resize to the correct size
          outPoints->points.resize(j);
        }

        outPoints->height = 1;
        outPoints->width  = static_cast<std::uint32_t>(j);
        
        outPoints->is_dense = true;

        pcl::toROSMsg(*outPoints, outMsg);
        if (this->time_synchronization_) {
            outMsg.header.stamp = scanMsg.header.stamp;
            if (deltaTime < 0) {
                outMsg.header.stamp.sec -= 1;
                outMsg.header.stamp.nanosec = 1e9 - deltaTime * 1e3;
            } else {
                outMsg.header.stamp.nanosec = deltaTime * 1e3;
            }
        } else {
            outMsg.header.stamp = this->get_clock()->now();
        }

        if(lastSec * 1e6 + lastNanosec > outMsg.header.stamp.sec * 1e6 + outMsg.header.stamp.nanosec){
            outMsg.header.stamp.sec++;
        }

        outMsg.header.stamp.sec = std::max((int64_t)outMsg.header.stamp.sec, lastSec);

        lastNanosec = outMsg.header.stamp.nanosec;
        lastSec = outMsg.header.stamp.sec;
        
        RCLCPP_INFO(this->get_logger(), "sec :%d , nanosec: %d ", outMsg.header.stamp.sec, outMsg.header.stamp.nanosec);
        
        this->pointCloudExPublisher_->publish(outMsg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_decoder::LidarDecoder)