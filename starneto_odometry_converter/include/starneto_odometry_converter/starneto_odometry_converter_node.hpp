#ifndef STARNETO_ODOMETRY_CONVERTER_NODE_H__
#define STARNETO_ODOMETRY_CONVERTER_NODE_H__

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>


#include <stdlib.h>
#include <string.h>
#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gpgga.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#define GZD_ID_SIZE 5
#define PRECISION_METER_SIZE 5
#define DEG_TO_RAD 0.017453292
#define GPS_HDOP_TO_COV 1.5f
#define RTK_HDOP_TO_COV 0.02f

using namespace std::chrono_literals;

namespace starneto_odometry_converter
{

using StarnetoOdometryConverterSyncPolicy = message_filters::sync_policies::ApproximateTime<starneto_msgs::msg::Gpfpd, starneto_msgs::msg::Gpgga>;

class StarnetoOdometryConverterNode : public rclcpp::Node
{
public:
    explicit StarnetoOdometryConverterNode(const rclcpp::NodeOptions & nodeOptions);
    ~StarnetoOdometryConverterNode() = default;

private:
    bool isInitialized_ = false;
    tf2::Stamped<tf2::Transform> relativePoseTransform_;
    GeographicLib::LocalCartesian relativeCartesia_;
    int mgrsPrecision_;
    std::string baseFrame_;
    std::vector<bool> allowStatusMap_;
    std::unique_ptr<message_filters::Subscriber<starneto_msgs::msg::Gpfpd>> gpfpdSubscriber_;
    std::unique_ptr<message_filters::Subscriber<starneto_msgs::msg::Gpgga>> gpggaSubscriber_;
    std::unique_ptr<message_filters::Synchronizer<StarnetoOdometryConverterSyncPolicy>> sync_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    void topicCallback(const starneto_msgs::msg::Gpfpd::ConstSharedPtr gpfpd, const starneto_msgs::msg::Gpgga::ConstSharedPtr gpgga);
};

}
#endif