#include "nightowl_remote_controller/h264_encoder.hpp"

extern "C" {
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
}

namespace nightowl_remote_controller
{

H264Encoder::H264Encoder(rclcpp::Node & node, size_t width, size_t height, size_t bitrate, double fps)
: node_(node)
{
    size_t size = width * height;
    /* 配置图片格式转换器 */
    imageRGBToYUVConverter_ = sws_getContext(width, height, AV_PIX_FMT_BGR24, width, height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, 0, 0, 0);
    /* 加载解码器 */
    codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec_) {
        RCLCPP_FATAL(node_.get_logger(), "Failed to find h264 codec!");
        return;
    }
    /* 分配内存 */
    codecContext_ = avcodec_alloc_context3(codec_);
    yuvFrame_ = av_frame_alloc();
    rgbFrame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!codecContext_ || !rgbFrame_  || !yuvFrame_ || !packet_) {
        RCLCPP_FATAL(node_.get_logger(), "Could not allocate memory");
        return;
    }
    /* 配置参数 */
    codecContext_->bit_rate = bitrate;
    codecContext_->rc_buffer_size = codecContext_->bit_rate;
    codecContext_->width = width;
    codecContext_->height = height;
    codecContext_->time_base = (AVRational){100, (int)(fps * 100.0f)};
    codecContext_->framerate = (AVRational){(int)(fps * 100.0f), 100};
    codecContext_->gop_size = (int)fps;
    codecContext_->has_b_frames   = 0;
    codecContext_->max_b_frames = 0;
    codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;
    rgbFrame_->format = AV_PIX_FMT_BGR24;
    rgbFrame_->width  = codecContext_->width;
    rgbFrame_->height = codecContext_->height;
    yuvFrame_->format = codecContext_->pix_fmt;
    yuvFrame_->width  = codecContext_->width;
    yuvFrame_->height = codecContext_->height;
    yuvFrame_->pts = -1; /* 这样初始值就是0了 */
    if (av_frame_get_buffer(yuvFrame_, 0) < 0) {
        RCLCPP_FATAL(node_.get_logger(), "Could not allocate the video frame data\n");
        return;
    }
    if (av_opt_set(codecContext_->priv_data, "profile", "baseline", 0) < 0 ||
        av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0) < 0 ||
        av_opt_set(codecContext_->priv_data, "preset","ultrafast", 0) < 0) {
        RCLCPP_FATAL(node_.get_logger(), "Failed to set h264 option!");
        return;
    }
    /* 打开解码器 */
    if (avcodec_open2(codecContext_, codec_, NULL) < 0) {
        RCLCPP_FATAL(node_.get_logger(), "Failed to open h264 codec!");
        return;
    }
}

H264Encoder::~H264Encoder()
{
    avcodec_close(codecContext_);
    av_frame_free(&yuvFrame_);
    av_frame_free(&rgbFrame_);
    av_packet_free(&packet_);
}

void H264Encoder::sendImage(std::vector<uint8_t> rgbImage)
{
    /* BGR转YUV420 */
    if (av_image_fill_arrays(rgbFrame_->data, rgbFrame_->linesize, rgbImage.data(), AV_PIX_FMT_BGR24, rgbFrame_->width, rgbFrame_->height, 1) < 0) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to convert image format");
        return;
    }
    sws_scale(imageRGBToYUVConverter_, rgbFrame_->data, rgbFrame_->linesize, 0, yuvFrame_->height, yuvFrame_->data, yuvFrame_->linesize);
    if (av_frame_make_writable(yuvFrame_) < 0) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to make a frame writable");
        return;
    }
    yuvFrame_->pts++;
    if (avcodec_send_frame(codecContext_, yuvFrame_) < 0) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to send a frame for encoding");
        return;
    }
}

std::pair<uint8_t*, size_t> H264Encoder::receiveImage()
{
    int ret = avcodec_receive_packet(codecContext_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        return std::make_pair((uint8_t*)NULL, 0);
    } else if (ret < 0) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to encode a frame");
        return std::make_pair((uint8_t*)NULL, 0);
    }
    return std::make_pair(packet_->data, packet_->size);
}

}  // namespace nightowl_remote_controller
