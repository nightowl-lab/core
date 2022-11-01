#ifndef SMARTER_EYE_CAMERA__SMARTER_EYE_CAMERA_NODE_H__
#define SMARTER_EYE_CAMERA__SMARTER_EYE_CAMERA_NODE_H__

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "smarter_eye_sdk/calibrationparams.h"
#include "smarter_eye_sdk/camerahandler.h"
#include "smarter_eye_sdk/stereocamera.h"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define SMARTER_EYE_DISPARITY_COUNT 81
#define SMARTER_EYE_PIXEL_SIZE 4.2
#define SMARTER_EYE_G_VALUE 9.8

using namespace std::chrono_literals;

namespace smarter_eye_camera
{

class SmarterEyeCameraNode : public rclcpp::Node, public CameraHandler
{
  public:
    SmarterEyeCameraNode(const rclcpp::NodeOptions & nodeOptions);

    void handleRawFrame(const RawImageFrame * rawFrame);

  private:
    bool useRosTimestamp_;                                                            /* 是否以数据处理时的ROS时间戳为时间 */
    int width_;                                                                       /* 图像宽度 */
    int height_;                                                                      /* 图像高度 */
    float imageRate_;                                                                 /* 图像帧率 */
    std::string ip_;                                                                  /* 摄像头IP地址 */
    std::string leftCameraInfoURL_;                                                   /* 左相机信息URL */
    std::string rightCameraInfoURL_;                                                  /* 右相机信息URL */
    bool useSDKCameraInfo_;                                                           /* 是否使用SDK返回的相机信息作为CameraInfo话题发布 */
    std::string baseFrame_;                                                           /* Frame ID */
    int imuAccelerationRange_;                                                        /* IMU加速度量程 */
    int imuRotationRange_;                                                            /* IMU角速度量程 */
    int imuRate_;                                                                     /* IMU读取频率 */
    std::shared_ptr<StereoCamera> camera_;                                            /* 摄像头实例 */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_; /* 点云发布者 */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;                /* IMU发布者 */
    bool isLookUpTableGenerated_ = false;                                             /* 查找表是否已经创建过 */
    StereoCalibrationParameters calibrationParams_;                                   /* 相机校准参数 */
    sensor_msgs::msg::Image leftImage_;                                               /* 左相机最后一帧画面 */
    sensor_msgs::msg::Image rightImage_;                                              /* 右相机最后一帧画面 */
    std::shared_ptr<image_transport::CameraPublisher> leftCameraPublisher_;           /* 左相机话题发布者 */
    std::shared_ptr<image_transport::CameraPublisher> rightCameraPublisher_;          /* 右相机话题发布者 */
    std::shared_ptr<camera_info_manager::CameraInfoManager> leftCameraInfoManager_;   /* 左相机信息管理器 */
    std::shared_ptr<camera_info_manager::CameraInfoManager> rightCameraInfoManager_;  /* 右相机信息管理器 */
    std::shared_ptr<float[]> xDistanceBuffer_;                                        /* 图像内点的X坐标 */
    std::shared_ptr<float[]> yDistanceBuffer_;                                        /* 图像内点的Y坐标 */
    std::shared_ptr<float[]> zDistanceBuffer_;                                        /* 图像内点的Z坐标 */
    std::shared_ptr<float[]> xLookUpTable_;                                           /* 点-X距离查找表 */
    std::shared_ptr<float[]> yLookUpTable_;                                           /* 点-Y距离查找表 */
    std::shared_ptr<float[]> zLookUpTable_;                                           /* 点-Z距离查找表 */
    std::shared_ptr<char[]> leftImageRGBBuffer_;                                      /* 左相机RGB缓冲区 */
    cv_bridge::CvImagePtr leftImageCvPtr_;                                             /* 左相机cv_bridge指针 */
    sensor_msgs::msg::PointCloud2 pcMsg_;                                             /* 点云消息 */

    /**
     * 初始化摄像头信息
     */
    void initCameraInfo();

    /**
     * 初始化话题所需要的一些公用不变部分
     */
    void initTopic();

    /**
     * 处理和发布点云数据
     * @param rawFrame 图像帧
     */
    void handlePointCloudGeneration(const RawImageFrame * rawFrame);

    /**
     * 处理和发布左摄像头数据
     * @param rawFrame 图像帧
     */
    void handleLeftCamera(const RawImageFrame * rawFrame);

    /**
     * 处理和发布右摄像头数据
     * @param rawFrame 图像帧
     */
    void handleRightCamera(const RawImageFrame * rawFrame);

    /**
     * 处理和发布IMU数据
     * @param rawFrame 图像帧
     */
    void handleIMU(const RawImageFrame * rawFrame);
};

}  // namespace smarter_eye_camera
#endif