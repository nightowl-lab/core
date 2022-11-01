#include "smarter_eye_camera/smarter_eye_camera_node.hpp"

#include "smarter_eye_camera/tool.hpp"
#include "smarter_eye_sdk/disparityconvertor.h"
#include "smarter_eye_sdk/frameext.h"
#include "smarter_eye_sdk/frameid.h"
#include "smarter_eye_sdk/motiondata.h"
#include "smarter_eye_sdk/satpext.h"
#include "smarter_eye_sdk/taskiddef.h"
#include "smarter_eye_sdk/yuv2rgb.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace smarter_eye_camera
{

SmarterEyeCameraNode::SmarterEyeCameraNode(const rclcpp::NodeOptions & nodeOptions) : Node("smarter_eye_camera_node", nodeOptions)
{
    /* 读取配置 */
    width_ = this->declare_parameter("image_width", 1280);
    height_ = this->declare_parameter("image_height", 720);
    ip_ = this->declare_parameter<std::string>("camera_ip");
    baseFrame_ = this->declare_parameter("base_frame", "front_stereo");
    useRosTimestamp_ = this->declare_parameter("use_ros_timestamp", true);
    useSDKCameraInfo_ = this->declare_parameter("use_sdk_camera_info", true);
    leftCameraInfoURL_ = this->declare_parameter("left_camera_info_url", "");
    rightCameraInfoURL_ = this->declare_parameter("right_camera_info_url", "");
    imageRate_ = this->declare_parameter("image_rate", 12.5);
    imuAccelerationRange_ = this->declare_parameter("imu_acceleration_range", 4);
    imuRotationRange_ = this->declare_parameter("imu_rotation_range", 500);
    imuRate_ = this->declare_parameter("imu_rate", 100);
    /* 检查配置 */
    CHECK_PARAM_IN_ARRAY_OR_THROW(imageRate_, ({12.5, 6.25, 4.17, 3.125, 2.5, 2.08, 1.79, 1.5625}), "image_rate is out of range!");
    CHECK_PARAM_IN_ARRAY_OR_THROW(imuAccelerationRange_, ({2, 4, 8, 16}), "imu_acceleration_range is out of range!");
    CHECK_PARAM_IN_ARRAY_OR_THROW(imuRotationRange_, ({250, 500, 1000, 2000}), "imu_rotation_range is out of range!");
    if (imuRate_ > 100 || imuRate_ < 10) throw new std::invalid_argument("imu_rate is out of range!");
    /* 摄像头初始化 */
    camera_.reset(StereoCamera::connect(ip_.c_str()));
    /* 等待摄像头连接 */
    rclcpp::Rate rate(1.0);
    while (!camera_->isConnected()) {
        if (!rclcpp::ok()) {
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for the camera to connect...");
        rate.sleep();
    }
    camera_->enableTasks(TaskId::DisplayTask);
    CHECK_SDK_STATUS_OR_THROW(camera_->setFrameRate(imageRate_), "Set FrameRate failed!");
    /* 获取参数 */
    CHECK_SDK_STATUS_OR_THROW(camera_->requestStereoCameraParameters(calibrationParams_), "Get Camera Parameters failed!");
    initCameraInfo();
    /* IMU初始化 */
    camera_->setImuAccelRange(imuAccelerationRange_);
    camera_->setImuRotationRange(imuRotationRange_);
    camera_->setImuReadFrequence(imuRate_);
    camera_->enableMotionData(true);
    /* 分配空间 */
    xDistanceBuffer_.reset(new float[width_ * height_]);
    yDistanceBuffer_.reset(new float[width_ * height_]);
    zDistanceBuffer_.reset(new float[width_ * height_]);
    leftImageRGBBuffer_.reset(new char[width_ * height_ * 3]);
    leftImageCvPtr_ = std::make_shared<cv_bridge::CvImage>();
    leftImageCvPtr_->encoding = "bgr8";
    /* 初始化发布者 */
    pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/point_cloud", 10);
    imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("output/imu", 10);
    leftCameraPublisher_ = std::make_shared<image_transport::CameraPublisher>(this, "output/left_camera");
    rightCameraPublisher_ = std::make_shared<image_transport::CameraPublisher>(this, "output/right_camera");
    /* 初始化部分话题通用部分 */
    initTopic();
    /* 启动推流,注意这个推流不是运行在主线程内的 */
    camera_->requestFrame(this, FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);
    RCLCPP_INFO(this->get_logger(), "smarter_eye_camera_node started.");
    /* 从这个地方开始,ros的主线程就和我们没关系了,剩下的部分都运行在sdk的线程里 */
}

void SmarterEyeCameraNode::initCameraInfo()
{
    leftCameraInfoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "left_camera", leftCameraInfoURL_);
    rightCameraInfoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "right_camera", rightCameraInfoURL_);
    if (useSDKCameraInfo_) {
        camera_info_manager::CameraInfo info;
        info.height = height_;
        info.width = width_;
        /* 填充P矩阵 */
        info.p[0] = calibrationParams_.focus;
        info.p[2] = calibrationParams_.cx;
        info.p[3] = calibrationParams_.Tx / 1000.0f;
        info.p[5] = calibrationParams_.focus;
        info.p[6] = calibrationParams_.cy;
        info.p[7] = calibrationParams_.Ty / 1000.0f;
        info.p[10] = 1;
        /* 填充R矩阵 */
        tf2::Matrix3x3 r;
        r.setRPY(calibrationParams_.RRoll, calibrationParams_.RPitch, calibrationParams_.RYaw);
        info.r[0] = r[0][0];
        info.r[1] = r[0][1];
        info.r[2] = r[0][2];
        info.r[3] = r[1][0];
        info.r[4] = r[1][1];
        info.r[5] = r[1][2];
        info.r[6] = r[2][0];
        info.r[7] = r[2][1];
        info.r[8] = r[2][2];
        /* 设置Info */
        leftCameraInfoManager_->setCameraInfo(info);
        rightCameraInfoManager_->setCameraInfo(info);
    }
}

void SmarterEyeCameraNode::initTopic()
{
    rightImage_.header.frame_id = baseFrame_;
    rightImage_.header.stamp = this->get_clock()->now();
    rightImage_.height = height_;
    rightImage_.width = width_;
    rightImage_.encoding = sensor_msgs::image_encodings::MONO8;
    rightImage_.step = width_ * 1;
    rightImage_.data.resize(width_ * height_ * 1);
}

void SmarterEyeCameraNode::handleRawFrame(const RawImageFrame * rawFrame)
{
    /* 注意本函数不是运行在主线程内的 */
    /* 帧信息处理 */
    if (rawFrame->index > 0) {
        handleIMU(rawFrame);
    }
    switch (rawFrame->frameId) {
        case FrameId::CalibLeftCamera:
            handleLeftCamera(rawFrame);
            break;
        case FrameId::CalibRightCamera:
            handleRightCamera(rawFrame);
            break;
        case FrameId::Disparity:
            handlePointCloudGeneration(rawFrame);
            break;
    }
}

void SmarterEyeCameraNode::handleIMU(const RawImageFrame * rawFrame)
{
    /* 注意本函数不是运行在主线程内的 */
    /* 解析IMU数据 */
    size_t size = rawFrame->index;
    const char * extended = (char *)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame);
    const FrameDataExtHead * header = (FrameDataExtHead *)extended;
    MotionData imuData;
    bool hasIMUData = false;
    if (header->dataType == FrameDataExtHead::Compound) {
        header = (const FrameDataExtHead *)header->data;
        size -= sizeof(FrameDataExtHead);
    }
    do {
        if (header->dataType == FrameDataExtHead::MotionData) {
            const MotionData * motionPtr = (const MotionData *)header->data;
            int totalNum = (header->dataSize - sizeof(FrameDataExtHead)) / sizeof(MotionData);
            /* 只保留最后一帧 */
            imuData = motionPtr[totalNum - 1];
            hasIMUData = true;
        }
        header = (const FrameDataExtHead *)((const char *)header + header->dataSize);
    } while ((const char *)header < (extended + size));
    /* 发布IMU数据 */
    if (hasIMUData) {
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = baseFrame_;
        if (useRosTimestamp_) {
            msg.header.stamp = this->get_clock()->now();
        }
        msg.angular_velocity.x = DEGREE_TO_RAD(imuData.gyroX);
        msg.angular_velocity.y = DEGREE_TO_RAD(imuData.gyroY);
        msg.angular_velocity.z = DEGREE_TO_RAD(imuData.gyroZ);
        msg.linear_acceleration.x = imuData.accelX * SMARTER_EYE_G_VALUE;
        msg.linear_acceleration.y = imuData.accelY * SMARTER_EYE_G_VALUE;
        msg.linear_acceleration.z = imuData.accelZ * SMARTER_EYE_G_VALUE;
        imuPublisher_->publish(msg);
    }
}

void SmarterEyeCameraNode::handleLeftCamera(const RawImageFrame * rawFrame)
{
    /* 注意本函数不是运行在主线程内的 */
    sensor_msgs::msg::CameraInfo info = std::move(leftCameraInfoManager_->getCameraInfo());
    YuvToRGB::YCbYCrPlannar2Rgb(rawFrame->image, leftImageRGBBuffer_.get(), width_, height_);
    cv::Mat rgbMat(rawFrame->height, rawFrame->width, CV_8UC3, leftImageRGBBuffer_.get());
    cv::cvtColor(rgbMat, leftImageCvPtr_->image, cv::COLOR_RGB2BGR);
    leftImageCvPtr_->toImageMsg(leftImage_);
    leftImage_.header.frame_id = baseFrame_;
    info.header.frame_id = baseFrame_;
    /* 时间填充 */
    if (useRosTimestamp_) {
        rclcpp::Time now = this->get_clock()->now();
        leftImage_.header.stamp = now;
        info.header.stamp = now;
    }
    leftCameraPublisher_->publish(leftImage_, info);
}

void SmarterEyeCameraNode::handleRightCamera(const RawImageFrame * rawFrame)
{
    /* 注意本函数不是运行在主线程内的 */
    sensor_msgs::msg::CameraInfo info = std::move(rightCameraInfoManager_->getCameraInfo());
    memcpy((char *)&rightImage_.data[0], rawFrame->image, width_ * height_ * 1);
    info.header.frame_id = baseFrame_;
    /* 时间填充 */
    if (useRosTimestamp_) {
        rclcpp::Time now = this->get_clock()->now();
        rightImage_.header.stamp = now;
        info.header.stamp = now;
    }
    rightCameraPublisher_->publish(rightImage_, info);
}

void SmarterEyeCameraNode::handlePointCloudGeneration(const RawImageFrame * rawFrame)
{
    /* 注意本函数不是运行在主线程内的 */
    /* 计算查找表 */
    int bitNum = DisparityConvertor::getDisparityBitNum(rawFrame->format);
    if (!isLookUpTableGenerated_) {
        xLookUpTable_.reset(new float[SMARTER_EYE_DISPARITY_COUNT * (int)pow(2, bitNum) * rawFrame->width]);
        yLookUpTable_.reset(new float[SMARTER_EYE_DISPARITY_COUNT * (int)pow(2, bitNum) * rawFrame->height]);
        zLookUpTable_.reset(new float[SMARTER_EYE_DISPARITY_COUNT * (int)pow(2, bitNum)]);
        DisparityConvertor::generateLookUpTableX(rawFrame->width, bitNum, calibrationParams_.Tx, calibrationParams_.cx, xLookUpTable_.get());
        DisparityConvertor::generateLookUpTableY(rawFrame->height, bitNum, calibrationParams_.Tx, calibrationParams_.cy, yLookUpTable_.get());
        DisparityConvertor::generateLookUpTableZ(bitNum, calibrationParams_.Tx, calibrationParams_.focus, zLookUpTable_.get());
        isLookUpTableGenerated_ = true;
    }
    /* 查表得到坐标 */
    DisparityConvertor::getWholeXDistanceByLookupTable(rawFrame->image, rawFrame->width, rawFrame->height, bitNum, xLookUpTable_.get(), xDistanceBuffer_.get());
    DisparityConvertor::getWholeYDistanceByLookupTable(rawFrame->image, rawFrame->width, rawFrame->height, bitNum, yLookUpTable_.get(), yDistanceBuffer_.get());
    DisparityConvertor::getWholeZDistanceByLookupTable(rawFrame->image, rawFrame->width, rawFrame->height, zLookUpTable_.get(), zDistanceBuffer_.get());
    /* 发布点云,彩色最大不允许误差超过2帧 */
    if (this->get_clock()->now() - leftImage_.header.stamp > 100ms) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Left Camera Image timeout when calculating Points Cloud!");
        return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = baseFrame_;
    if (useRosTimestamp_) {
        msg.header.stamp = this->get_clock()->now();
    }
    msg.height = height_;
    msg.width = width_;
    msg.point_step = 16;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "rgb";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.data.resize(msg.height * msg.width * 16);
    float badPoint = std::numeric_limits<float>::quiet_NaN();
    int pointCount = width_ * height_;
    for (int i = 0; i < pointCount; i++) {
        /* 这里的XYZ是摄像头自带相机坐标系的数据 */
        float x = xDistanceBuffer_[i] / 1000.0f;
        float y = yDistanceBuffer_[i] / 1000.0f;
        float z = zDistanceBuffer_[i] / 1000.0f;
        if (fabs(x) > 200.0f || fabs(y) > 200.0f || fabs(z) > 200.0f) {
            memcpy(&msg.data[0 + 16 * i], &badPoint, 4);
            memcpy(&msg.data[4 + 16 * i], &badPoint, 4);
            memcpy(&msg.data[8 + 16 * i], &badPoint, 4);
            memcpy(&msg.data[12 + 16 * i], &badPoint, 4);
        } else {
            int rgb = leftImage_.data[3 * i + 2] | (leftImage_.data[3 * i + 1] << 8) | (leftImage_.data[3 * i + 0] << 16);
            memcpy(&msg.data[0 + 16 * i], &x, 4);
            memcpy(&msg.data[4 + 16 * i], &y, 4);
            memcpy(&msg.data[8 + 16 * i], &z, 4);
            memcpy(&msg.data[12 + 16 * i], &rgb, 4);
        }
    }
    pointCloudPublisher_->publish(std::move(msg));
}

}  // namespace smarter_eye_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smarter_eye_camera::SmarterEyeCameraNode)