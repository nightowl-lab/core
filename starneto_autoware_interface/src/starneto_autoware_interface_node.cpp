#include "starneto_autoware_interface/starneto_autoware_interface_node.hpp"

namespace starneto_autoware_interface
{

StarnetoAutowareInterfaceNode::StarnetoAutowareInterfaceNode(const rclcpp::NodeOptions& nodeOptions)
    : Node("starneto_autoware_interface", nodeOptions)
{
    this->baseFrame_ = this->declare_parameter("base_frame", "base_link", rcl_interfaces::msg::ParameterDescriptor(), true);

    this->gpfpdSubscriber_ = this->create_subscription<starneto_msgs::msg::Gpfpd>("input_topic_gpfpd", 10, [&](const starneto_msgs::msg::Gpfpd::SharedPtr msg){
        this->gpfpdMessage_ = *msg;
   });

    this->gtimuSubscriber_ = this->create_subscription<starneto_msgs::msg::Gtimu>("input_topic_gtimu", 10, [&](const starneto_msgs::msg::Gtimu::SharedPtr msg){
        this->gtimuxMessage_ = *msg;
    });

    this->gpggaSubscriber_ = this->create_subscription<starneto_msgs::msg::Gpgga>("input_topic_gpgga", 10, [&](const starneto_msgs::msg::Gpgga::SharedPtr msg){
        this->gpggaMessage_ = *msg;
    });

    this->navSatFixPublisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    this->navPvtPublisher_ = this->create_publisher<ublox_msgs::msg::NavPVT>("navpvt", 10);
    this->imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    this->timer_ = this->create_wall_timer(10ms, std::bind(&StarnetoAutowareInterfaceNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "\033[1;32m<--Starneto_autoware_interface_node started.-->\033[0m");
}

void StarnetoAutowareInterfaceNode::timerCallback()
{
    toAutowareNavSatFixMessage();
    toAutowareNavPvtMessage();
    toAutowareImuMessage();
}

void StarnetoAutowareInterfaceNode::toAutowareNavSatFixMessage()
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "ins_link";
    msg.status.status = 0;
    msg.status.service = 15;

    if(!this->gpggaMessage_.status)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GPS Initialization Not Finish Yet");

    // msg.latitude = this->gpfpdMessage_.latitude;
    // msg.longitude = this->gpfpdMessage_.longitude;

    // 采用GPGGA的经纬度信息
    msg.latitude = this->gpggaMessage_.latitude;
    msg.longitude = this->gpggaMessage_.longitude;

    // GPGGA 没有高度信息, 只能用GPFPD的高度信息
    msg.altitude = this->gpfpdMessage_.altitude;

    /* 计算协方差 */
    double cov = std::pow(this->gpggaMessage_.hdop * ((this->gpggaMessage_.status == 4 || this->gpggaMessage_.status == 5) 
        ? RTK_HDOP_TO_COV : GPS_HDOP_TO_COV), 2);
    
    msg.position_covariance.at(0) = cov;
    msg.position_covariance.at(4) = cov;
    msg.position_covariance.at(8) = cov;

    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    this->navSatFixPublisher_->publish(msg);
}

void StarnetoAutowareInterfaceNode::toAutowareNavPvtMessage()
{
    ublox_msgs::msg::NavPVT msg;

    msg.fix_type = 3;
    msg.flags = 3;
    msg.flags2 = 234;
    

    msg.heading = DEGREE_TO_RAD(this->gpfpdMessage_.heading ) / 1e-5;   //Heading of motion 2-D [deg / 1e-5]
    msg.num_sv = this->gpfpdMessage_.nsv1 + this->gpfpdMessage_.nsv2;

    // msg.lat = this->gpfpdMessage_.latitude / 1e-7;                      // [deg] to [deg / 1e-7]
    // msg.lon = this->gpfpdMessage_.longitude / 1e-7;                     // [deg] to [deg / 1e-7]

    // 采用GPGGA的经纬度信息
    msg.lat = this->gpggaMessage_.latitude / 1e-7;                      // [deg] to [deg / 1e-7]
    msg.lon = this->gpggaMessage_.longitude / 1e-7;                     // [deg] to [deg / 1e-7]
    msg.h_msl = this->gpfpdMessage_.altitude  * 1000;                   // [m] to [mm]
    msg.vel_e = this->gpfpdMessage_.ve * 1000;                          // [m/s] to [mm/s]
    msg.vel_n = this->gpfpdMessage_.vn * 1000;                          // [m/s] to [mm/s]
    msg.vel_n = this->gpfpdMessage_.vn * 1000;                          // [m/s] to [mm/s]

    msg.head_veh = 0;
    msg.mag_dec = 0;
    msg.mag_acc = 0;

    this->navPvtPublisher_->publish(msg);
}

void StarnetoAutowareInterfaceNode::toAutowareImuMessage()
{
    // TODO
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "ins_link";

    //[degree/s] to [rad/s]
    msg.angular_velocity.x = DEGREE_TO_RAD(this->gtimuxMessage_.gx);
    msg.angular_velocity.y = DEGREE_TO_RAD(this->gtimuxMessage_.gy);
    msg.angular_velocity.z = DEGREE_TO_RAD(this->gtimuxMessage_.gz);

    //[g] to [m/s^2]
    msg.linear_acceleration.x = this->gtimuxMessage_.ax * G;
    msg.linear_acceleration.y = this->gtimuxMessage_.ay * G;
    msg.linear_acceleration.z = this->gtimuxMessage_.az * G;

    tf2::Quaternion quaternion;
    //[rpy] to [quaternion]
    quaternion.setRPY(  DEGREE_TO_RAD(this->gpfpdMessage_.roll),
                        -DEGREE_TO_RAD(this->gpfpdMessage_.pitch), 
                        DEGREE_TO_RAD(this->gpfpdMessage_.heading));

    //quaternion = quaternion.normalize();

    msg.orientation.x = quaternion.x();
    msg.orientation.y = quaternion.y();
    msg.orientation.z = quaternion.z();
    msg.orientation.w = quaternion.w();

    this->imuPublisher_->publish(msg);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(starneto_autoware_interface::StarnetoAutowareInterfaceNode)