#include <angles/angles.h>
#include <yaml-cpp/yaml.h>

#include "lidar_decoder/lidar_rawdata.hpp"

namespace lidar_rawdata
{
    RawData::RawData(rclcpp::Node::SharedPtr node_ptr)
        : node_ptr_(node_ptr)
    {
        this->is_init_angle_ = false;
        this->is_init_curve_ = false;
        this->is_init_top_fw_ = false;
    }

    void RawData::loadConfigFile()
    {
        std::string model;
        std::string resolution_param;

        this->start_angle_      = this->node_ptr_->declare_parameter("start_angle", float(0));
        this->end_angle_        = this->node_ptr_->declare_parameter("end_angle", float(360));
        this->distance_unit_    = this->node_ptr_->declare_parameter("distance_unit", float(0.25));
        this->calibration_file_ = this->node_ptr_->declare_parameter("calibration_file", std::string(""));
        this->scan_start_angle_ = this->node_ptr_->declare_parameter("scan_start_angle", float(0));
        this->scan_end_angle_   = this->node_ptr_->declare_parameter("scan_end_angle", float(36000));

        if (this->start_angle_ < 0 || this->start_angle_ > 360 || this->end_angle_ < 0 || this->end_angle_ > 360)
        {
            this->start_angle_ = 0;
            this->end_angle_ = 360;

            RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "start angle and end angle select feature deactivated.");
        } 
        else 
        {
            RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(),"start angle and end angle select feature activated.");
        }

        this->angle_flag_ = true;
        if (this->start_angle_ > this->end_angle_)
        {
            this->angle_flag_ = false;
            RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "Start angle is smaller than end angle, not the normal state!");
        }
        
        RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "start_angle: " << this->start_angle_ << " end_angle: " << this->end_angle_ << " angle_flag: " << this->angle_flag_);

        this->start_angle_  = this->start_angle_ / 180 * M_PI;
        this->end_angle_    = this->end_angle_ / 180 * M_PI;

        this->max_distance_     = this->node_ptr_->declare_parameter("max_distance", 200.0f);
        this->min_distance_     = this->node_ptr_->declare_parameter("min_distance", 0.2f);
        this->cbMethod_         = this->node_ptr_->declare_parameter("cbMethod", true);
        this->return_mode_      = this->node_ptr_->declare_parameter("return_mode", 1);
        this->config_vert_      = this->node_ptr_->declare_parameter("config_vert", true);
        this->print_vert_       = this->node_ptr_->declare_parameter("print_vert", true);
        this->config_vert_file_ = this->node_ptr_->declare_parameter("config_vert_file", false);
        
        RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "distance threshlod, max: " << max_distance_ << ", min: " << min_distance_);
        RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "return mode : " << return_mode_);

        intensity_mode_ = 1;
        info_print_flag_ = false;
        config_vert_angle = false;

        model = this->node_ptr_->declare_parameter("model", std::string("LSC16"));
        numOfLasers = 16;
        R1_ = 0.04376;   //calibration
        R2_ = 0.010875;

        intensityFactor = 51;

        //return mode default
        //return_mode_ = 1;

        //Vertical Angle Calibration for device package
        for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) 
        {
            cos_scan_altitude_caliration[i] = std::cos(scan_altitude_original[i]);
            sin_scan_altitude_caliration[i] = std::sin(scan_altitude_original[i]);
            scan_altitude[i] = scan_altitude_original[i];
        }

#if 0
        //Vertical Angle Calibration for file
        if(config_vert_file_){
          YAML::Node calibration_config = YAML::LoadFile(calibration_file_);
          for(int i = 0; i < LSC16_SCANS_PER_FIRING; i++){
            vert_angle = calibration_config["lasers"][i]["vert_correction"].as<double>();
            cos_scan_altitude_caliration[i] = std::cos(vert_angle*DEG_TO_RAD);
            sin_scan_altitude_caliration[i] = std::sin(vert_angle*DEG_TO_RAD);
          }
        }
#endif


        for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) 
        {
            float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
            cos_azimuth_table[rot_index] = cosf(rotation);
            sin_azimuth_table[rot_index] = sinf(rotation);
        }

        // receive difop data
        // subscribe to difop lslidar packets, if not right correct data in difop, it will not revise the correct data in the
        // VERT_ANGLE, HORI_ANGLE etc.
        rclcpp::Node::SharedPtr raw_node(this->node_ptr_);
        this->difop_sub_ = raw_node->create_subscription<lidar_msgs::msg::LidarPacket>("input/difop_packet", 10, std::bind(&RawData::processDifop, this, std::placeholders::_1));
    }

    void RawData::processDifop(const lidar_msgs::msg::LidarPacket& difop_msg) 
    {
        // std::cout << "Enter difop callback!" << std::endl;
        const uint8_t *data = &difop_msg.data[0];
        bool is_support_dual_return = false;

        // check header
        if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a) 
        {
            return;
        }

        int version_data = data[1202];
        if (config_vert_) 
        {
            if (2 == version_data)
            {
                for (int i = 0; i < 16; i++) 
                {
                    uint8_t data1 = data[234 + 2 * i];
                    uint8_t data2 = data[234 + 2 * i + 1];
                    int vert_angle = data1 * 256 + data2;
                    if (vert_angle > 32767) 
                    {
                        vert_angle = vert_angle - 65535;
                    }

                    scan_altitude[i] = ((float) vert_angle / 100.f) * DEG_TO_RAD;

                    if (scan_altitude[i] != 0) 
                    {
                        if (fabs(scan_altitude_original[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) 
                        {
                            scan_altitude[i] = scan_altitude_original[i];
                        }
                    } 
                    else 
                    {
                        scan_altitude[i] = scan_altitude_original[i];
                    }
                    config_vert_angle = true;
                }
            }
            else
            {
                for (int i = 0; i < 16; i++) 
                {
                    uint8_t data1 = data[245 + 2 * i];
                    uint8_t data2 = data[245 + 2 * i + 1];
                    int vert_angle = data1 * 256 + data2;
                    if (vert_angle > 32767) 
                    {
                        vert_angle = vert_angle - 65535;
                    }

                    scan_altitude[i] = ((float) vert_angle / 100.f) * DEG_TO_RAD;

                    if (scan_altitude[i] != 0) 
                    {
                        if (fabs(scan_altitude_original[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) 
                        {
                            scan_altitude[i] = scan_altitude_original[i];
                        }
                    } 
                    else 
                    {
                        scan_altitude[i] = scan_altitude_original[i];
                    }

                    config_vert_angle = true;
                }
            }
        }

        // rpm
        if ((data[8] == 0x04) && (data[9] == 0xB0)) 
        {
            rpm_ = 1200;
        } else if ((data[8] == 0x02) && (data[9] == 0x58)) 
        {
            rpm_ = 600;
        } else if ((data[8] == 0x01) && (data[9] == 0x2C)) 
        {
            rpm_ = 300;
        } 
        else 
        {
            //ROS_WARN("Invalid motor rpm!");
        }
        if (print_vert_) 
        {
            //ROS_INFO("rpm is %d", rpm_);
        }
    }

    void
    RawData::unpack(const lidar_msgs::msg::LidarPacket &pkt, pcl::PointCloud<lslidar_pointcloud::PointXYZIRADT>::Ptr pointcloud, double deltaTime) 
    {
        float azimuth;  // 0.01 dgree
        float intensity;
        float azimuth_diff;
        float azimuth_corrected_f;
        int azimuth_corrected;
        uint32_t packetEndTimestamp = 0;
        memcpy(&packetEndTimestamp, &pkt.data[1200], 4);
        deltaTime = packetEndTimestamp - deltaTime;
        if (deltaTime < 0) deltaTime += 1e6;
        

        // float time_diff_start_to_this_packet = (pkt.stamp.sec - scan_start_time).seconds();


        if (config_vert_angle) 
        {
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) 
            {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude[i]);

                if (print_vert_) 
                {
                    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Channel %d Data  %f", i, scan_altitude[i] * RAD_TO_DEG);
                }
            }

            config_vert_angle = false;
        }

        const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
        for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
        {

            if (UPPER_BANK != raw->blocks[block].header) 
            {
                rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                RCLCPP_INFO_STREAM_THROTTLE(this->node_ptr_->get_logger(), steady_clock, 180, "skipping LSLIDAR DIFOP packet");
                break;
            }

            azimuth = (float) (256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);


            if (2 == return_mode_) 
            {

                if (block < (BLOCKS_PER_PACKET - 2))  // 12
                {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block + 2].rotation_2 + raw->blocks[block + 2].rotation_1;
                    azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                } else {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azi2 = 256 * raw->blocks[block - 2].rotation_2 + raw->blocks[block - 2].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                }

            } 
            else 
            {
                if (block < (BLOCKS_PER_PACKET - 1))  // 12
                {
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
                    azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                } else {
                    //block 12
                    int azi1, azi2;
                    azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
                    azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);
                }
            }

            float cos_azimuth;
            float sin_azimuth;
            for (int firing = 0, k = 0; firing < LSC16_FIRINGS_PER_BLOCK; firing++)  // 2
            {
                for (int dsr = 0; dsr < LSC16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
                {
                    azimuth_corrected_f = azimuth + azimuth_diff / (LSC16_SCANS_PER_FIRING * 2) *
                                                    (LSC16_SCANS_PER_FIRING * firing + dsr);

                    azimuth_corrected = ((int) round(azimuth_corrected_f)) % 36000;  // convert to integral value...

                    cos_azimuth = cos_azimuth_table[azimuth_corrected];
                    sin_azimuth = sin_azimuth_table[azimuth_corrected];

                    //distance
                    union two_bytes tmp;
                    tmp.bytes[0] = raw->blocks[block].data[k];
                    tmp.bytes[1] = raw->blocks[block].data[k + 1];
                    int distance = tmp.uint;

                    // read intensity
                    intensity = raw->blocks[block].data[k + 2];
                    float distance2 = (distance * DISTANCE_RESOLUTION) * distance_unit_;

                    //The offset calibration
                    float arg_horiz = (float) azimuth_corrected_f * ROTATION_RESOLUTION;
                    arg_horiz = arg_horiz > 360 ? (arg_horiz - 360) : arg_horiz;
                    float arg_horiz_orginal = (14.67 - arg_horiz) * M_PI / 180;

                    lslidar_pointcloud::PointXYZIRADT point;
                    if ((scan_start_angle_ < azimuth_corrected_f) && (azimuth_corrected_f < scan_end_angle_)) 
                    {
                        if (distance2 > max_distance_ || distance2 < min_distance_) 
                        {
                            point.x = NAN;
                            point.y = NAN;
                            point.z = NAN;
                            point.return_type = 0;
                            point.azimuth = 0;
                            point.distance = 0;
                            point.intensity = 0;
                            point.time_stamp = 0;
                            point.ring = 0;
                            pointcloud->at(2 * this->block_num + firing, dsr) = point;
                        } 
                        else 
                        {
                            float x, y, z;
                            if (cbMethod_)
                            {
                                x = distance2 * cos_scan_altitude_caliration[dsr] * cos_azimuth +
                                          R1_ * cos(arg_horiz_orginal);
                                y = -distance2 * cos_scan_altitude_caliration[dsr] * sin_azimuth +
                                          R1_ * sin(arg_horiz_orginal);
                                z = distance2 * sin_scan_altitude_caliration[dsr] + 0.426 / 100.f;
                            } 
                            else
                            {
                                x = distance2 * cos_scan_altitude_caliration[dsr] * cos_azimuth;
                                y = -distance2 * cos_scan_altitude_caliration[dsr] * sin_azimuth;
                                z = distance2 * sin_scan_altitude_caliration[dsr];
                            }

                            point.x = x;
                            point.y = y;
                            point.z = z; 
                            point.return_type = 0;
                            point.ring =  dsr;
                            point.azimuth = azimuth;
                            point.distance = distance;
                            point.intensity = intensity;
                            point.time_stamp = (deltaTime - LSC16_BLOCK_TDURATION * (12 - (block + 1))) - ((firing == 0 ? 16 : 0) + 15 - dsr) * LSC16_DSR_TOFFSET;
                            point.time_stamp *= 1e-6;
                            
                            // std::cout << azimuth << std::endl;

                            // float time = 0;

                            // if(timing_offsets.size())
                            // {
                            //     time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;
                            // }

                            // point.time_stamp = this->node_ptr_->get_clock()->now().seconds();
                            //RCLCPP_INFO(this->node_ptr_->get_logger(), "timestamp: %lf", time);
                            pointcloud->at(2 * this->block_num + firing, dsr) = point;
                        }
                    } 
                    else 
                    {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                        point.return_type = 0;
                        point.azimuth = 0;
                        point.distance = 0;
                        point.intensity = 0;
                        point.ring = 0;
                        pointcloud->at(2 * this->block_num + firing, dsr) = point;
                    }
                }
            }
        }
    }
}
