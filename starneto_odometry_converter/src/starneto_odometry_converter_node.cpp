#include "starneto_odometry_converter/starneto_odometry_converter_node.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace starneto_odometry_converter
{

StarnetoOdometryConverterNode::StarnetoOdometryConverterNode(const rclcpp::NodeOptions & nodeOptions)
    : Node("starneto_odometry_converter_node", nodeOptions), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
{
    this->baseFrame_ = this->declare_parameter("base_frame", "base_link");
    rclcpp::Parameter param("allow_status_map", std::vector<bool>({}));
    this->declare_parameter("allow_status_map", std::vector<bool>({}));
    this->get_parameter("allow_status_map", param);
    allowStatusMap_ = param.as_bool_array();
    this->odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>("output/odometry", 10);
    this->gpfpdSubscriber_ = std::make_unique<message_filters::Subscriber<starneto_msgs::msg::Gpfpd>>(this, "input/gpfpd");
    this->gpggaSubscriber_ = std::make_unique<message_filters::Subscriber<starneto_msgs::msg::Gpgga>>(this, "input/gpgga");
    this->sync_ = std::make_unique<message_filters::Synchronizer<StarnetoOdometryConverterSyncPolicy>>(StarnetoOdometryConverterSyncPolicy(10), *this->gpfpdSubscriber_, *this->gpggaSubscriber_);
    this->sync_->registerCallback(std::bind(&StarnetoOdometryConverterNode::topicCallback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "starneto_odometry_converter_node started");
}

void StarnetoOdometryConverterNode::topicCallback(const starneto_msgs::msg::Gpfpd::ConstSharedPtr gpfpd, const starneto_msgs::msg::Gpgga::ConstSharedPtr gpgga)
{
    /* 计算旋转四元数 */
    tf2::Quaternion tempQuat;
    tempQuat.setRPY(gpfpd->roll * DEG_TO_RAD , gpfpd->pitch * DEG_TO_RAD, 2 * M_PI - gpfpd->heading * DEG_TO_RAD);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(tempQuat);
    if (!this->allowStatusMap_[gpgga->status & 0xF]) {
        return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation = quat;
    /* 变换相对坐标系 */
    if (!this->isInitialized_) {
        tf2::Transform relativeTransform;
        relativeTransform.setRotation(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w));
        relativePoseTransform_.setData(relativeTransform.inverse());
        relativeCartesia_ = GeographicLib::LocalCartesian(gpgga->latitude, gpgga->longitude, gpgga->msl);
        this->isInitialized_ = true;
        /* 打日记专用 */
        RCLCPP_INFO(this->get_logger(), "System Initialized, latitude: %f, longitude: %f, height: %f, roll: %f, pitch: %f, yaw: %f", gpgga->latitude, gpgga->longitude, gpgga->msl, gpfpd->roll * DEG_TO_RAD , gpfpd->pitch * DEG_TO_RAD, gpfpd->heading * DEG_TO_RAD);
    }
    relativeCartesia_.Forward(gpgga->latitude, gpgga->longitude, gpgga->msl, pose.pose.position.y, pose.pose.position.x, pose.pose.position.z);
    pose.pose.position.y = -pose.pose.position.y;
    tf2::doTransform(pose, pose, tf2::toMsg((relativePoseTransform_)));
    /* 变换车辆坐标系 */
    try {
        geometry_msgs::msg::TransformStamped transformStamped = tfBuffer_.lookupTransform(baseFrame_, gpgga->header.frame_id, this->get_clock()->now());
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        tf2::doTransform(pose, pose, transformStamped);
    } catch (tf2::LookupException) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for tf: %s -> %s", baseFrame_.c_str(), gpgga->header.frame_id.c_str());
        return;
    }

    /* 发布信息 */
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = gpgga->header.stamp;
    odom.header.frame_id = "odom";
    odom.pose.pose = pose.pose;
    /* 计算协方差 */
    double cov = std::pow(gpgga->hdop * ((gpgga->status == 4 || gpgga->status == 5) ? RTK_HDOP_TO_COV : GPS_HDOP_TO_COV), 2);
    odom.pose.covariance.at(0) = cov;
    odom.pose.covariance.at(7) = cov;
    odom.pose.covariance.at(14) = cov;
    this->odomPublisher_->publish(std::move(odom));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(starneto_odometry_converter::StarnetoOdometryConverterNode)
