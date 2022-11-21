#ifndef NIGHTOWL_REMOTE_CONTROLLER__H264_ENCODER_H__
#define NIGHTOWL_REMOTE_CONTROLLER__H264_ENCODER_H__

#include <rclcpp/rclcpp.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}
#include <sensor_msgs/msg/image.hpp>

#define LIAVCODEC_OUTPUT_BUFFER_LENGTH 1024000

namespace nightowl_remote_controller
{

class H264Encoder
{
  public:
    /**
     * Construct a new H264Encoder object
     * 
     * @param node ROS实例
     * @param width 图像宽度,单位像素
     * @param height 图像高度,单位像素
     * @param bitrate 图像宽度,单位bits/s
     * @param fps 帧率
     */
    H264Encoder(rclcpp::Node & node, size_t width, size_t height, size_t bitrate, double fps);

    /**
     * 获取一包H264数据
     *
     * @return std::pair<uint8_t *, size_t> H264包指针和大小
     */
    std::pair<uint8_t *, size_t> receiveImage();

    /**
     * 发送1帧图像到编码器
     *
     * @param rgbImage RGB排列字节数组
     */
    void sendImage(std::vector<uint8_t> rgbImage);

    ~H264Encoder();

  private:
    rclcpp::Node & node_;                        /* ROS节点 */
    struct SwsContext * imageRGBToYUVConverter_; /* RGB格式图像转YUV格式实例 */
    AVCodec * codec_;                            /* 编码器 */
    AVCodecContext * codecContext_ = NULL;       /* 编码器实例 */
    AVFrame * rgbFrame_;                         /* RGB图像帧 */
    AVFrame * yuvFrame_;                         /* YUV图像帧 */
    AVPacket * packet_;                          /* H264包 */
};

}  // namespace nightowl_remote_controller

#endif