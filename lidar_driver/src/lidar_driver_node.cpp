#include <signal.h>
#include "lidar_driver/lidar_driver_node.hpp"

volatile sig_atomic_t flag = 1;

using namespace std::chrono_literals;

namespace lidar_driver
{
    static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;    // 3200000 / 16
    static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

    LidarDriver::LidarDriver(const rclcpp::NodeOptions& nodeOptions) 
        : Node("lidar_driver", nodeOptions)
    {
        this->config_.frame_id  = this->declare_parameter("frame_id", std::string("lidar_link"));
        this->config_.model     =  this->declare_parameter("model", std::string("LSC16"));

        //packet frequency (Hz)
        double packet_rate = 840; //20000 / 24

        this->config_.rpm = this->declare_parameter("rpm", 600.0);
        this->config_.return_mode = this->declare_parameter("return_mode", 1);

        double frequency = (this->config_.rpm / 60.0);

        int npackets = (int)ceil(packet_rate / frequency);

        this->config_.npackets = this->declare_parameter("npackets", npackets);

        RCLCPP_INFO_STREAM(this->get_logger(), "publishing " << this->config_.npackets << " packets per scan");

        std::string dump_file;
        dump_file = this->declare_parameter("pcap", std::string(""));

        this->msop_udp_port = this->declare_parameter("msop_port", (int)MSOP_DATA_PORT_NUMBER);
        this->difop_udp_port = this->declare_parameter("difop_port", (int)DIFOP_DATA_PORT_NUMBER);

        if(dump_file == "")
        {
            rclcpp::Node::SharedPtr node_(this);
            this->msop_input_.reset(new lidar_driver::InputSocket(node_, this->msop_udp_port));
            this->difop_input_.reset(new lidar_driver::InputSocket(node_, this->difop_udp_port));
        }

        this->msopPublisher_   = this->create_publisher<lidar_msgs::msg::LidarScanUnified>("output/msop_packet", 10);
        this->difopPublisher_  = this->create_publisher<lidar_msgs::msg::LidarPacket>("output/difop_packet", 10);

        this->difop_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&LidarDriver::difopPoll, this)));

        this->time_synchronization_ = this->declare_parameter("time_synchronization", false);

        if(this->time_synchronization_)
        {
            this->synchronizationPublisher_ = this->create_publisher<sensor_msgs::msg::TimeReference>("output/sync_header", 1);
        }

        this->timer_ = this->create_wall_timer(10ms, std::bind(&LidarDriver::msopPoll, this));
        RCLCPP_INFO(this->get_logger(), "\033[1;32m<--lidar_driver_node started.-->\033[0m");
    }

    LidarDriver::~LidarDriver()
    {
        if(this->difop_thread_ != nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR!!!");
            this->difop_thread_->join();
        }
    }

    bool LidarDriver::msopPoll(void)
    {
        //Small-endian mode
        // RCLCPP_INFO(this->get_logger(), "msop package poll");

        lidar_msgs::msg::LidarScanUnified::Ptr scan(new lidar_msgs::msg::LidarScanUnified);

        int mode = this->config_.return_mode;

        uint64_t GPSCurrentTS;

        if(this->difop_input_->getUpdateFlag())
        {
            int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND / BLOCKS_ONE_CHANNEL_PER_PKT);
            packets_rate = ceil(packets_rate / 2);
            this->config_.rpm = this->difop_input_->getRpm();

            this->config_.npackets = ceil(packets_rate * 60 / this->config_.rpm) * mode;

            this->difop_input_->clearUpdateFlag();

            RCLCPP_DEBUG(this->get_logger(), "packet rate is %d, rpm is %3.3f, npacket is %d", packets_rate, this->config_.rpm, this->config_.npackets);
        }

        scan->packets.resize(this->config_.npackets);

        for(int i = 0; i < this->config_.npackets; i++)
        {
            while(true)
            {
                int rc = this->msop_input_->getPacket(&scan->packets[i], this->config_.time_offset);

                if(rc == 0)
                {
                    break;
                }
                
                if(rc < 0)
                {
                    return false;
                }
            }

            if(i == 0)
            {
                GPSCurrentTS = this->GPSCountingTS;
            }
        }

        if(this->time_synchronization_)
        {
            sensor_msgs::msg::TimeReference sync_header;

            lidar_msgs::msg::LidarPacket pkt = scan->packets[0];
            uint64_t packet_timestamp;

            packet_timestamp = (pkt.data[1200]  + pkt.data[1201] * pow(2, 8) + pkt.data[1202] * pow(2, 16) + pkt.data[1203] * pow(2, 24)) * 1e3; //ns
            
            this->timeStamp = rclcpp::Time(GPSCurrentTS, packet_timestamp); //s , ns

            sync_header.header.stamp = timeStamp;
            scan->header.stamp = timeStamp;

            this->synchronizationPublisher_->publish(sync_header);
        }

        if(this->time_synchronization_)
        {
            scan->header.frame_id = this->config_.frame_id;
        }
        else
        {
            scan->header.stamp = scan->packets.back().stamp;
        }

        scan->header.frame_id = this->config_.frame_id;
        this->msopPublisher_->publish(*scan);

        return true;
    }

    void LidarDriver::difopPoll(void)
    {
        //Big-endian mode
        // RCLCPP_INFO(this->get_logger(), "difop package poll");

        lidar_msgs::msg::LidarPacket::Ptr difop_packet_ptr(new lidar_msgs::msg::LidarPacket());

        while(rclcpp::ok())
        {
            lidar_msgs::msg::LidarPacket difop_packet_msg;
            int rc = this->difop_input_->getPacket(&difop_packet_msg, this->config_.time_offset);

            if(rc == 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "Publishing a difop data");
                *difop_packet_ptr = difop_packet_msg;
                this->difopPublisher_->publish(*difop_packet_ptr);

                int version_data = difop_packet_msg.data[1202];
                if(2 == version_data)
                {
                    this->packetTimeStamp[4] = difop_packet_msg.data[41];
                    this->packetTimeStamp[5] = difop_packet_msg.data[40];
                    this->packetTimeStamp[6] = difop_packet_msg.data[39];
                    this->packetTimeStamp[7] = difop_packet_msg.data[38];
                    this->packetTimeStamp[8] = difop_packet_msg.data[37];
                    this->packetTimeStamp[9] = difop_packet_msg.data[36];
                } 
                else
                {
                    this->packetTimeStamp[4] = difop_packet_msg.data[57];
                    this->packetTimeStamp[5] = difop_packet_msg.data[56];
                    this->packetTimeStamp[6] = difop_packet_msg.data[55];
                    this->packetTimeStamp[7] = difop_packet_msg.data[54];
                    this->packetTimeStamp[8] = difop_packet_msg.data[53];
                    this->packetTimeStamp[9] = difop_packet_msg.data[52];
                }

                struct tm cur_time;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec     = this->packetTimeStamp[4];
                cur_time.tm_min     = this->packetTimeStamp[5];
                cur_time.tm_hour    = this->packetTimeStamp[6] + 8;
                cur_time.tm_mday    = this->packetTimeStamp[7];
                cur_time.tm_mon     = this->packetTimeStamp[8] - 1;
                cur_time.tm_year    = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = mktime(&cur_time);

                if (GPSCountingTS != this->pointcloudTimeStamp)
                {
                    cnt_gps_ts = 0;
                    GPSCountingTS = this->pointcloudTimeStamp;
                }
                else if (cnt_gps_ts == 3)
                {
                    GPSStableTS = GPSCountingTS;
                }
                else
                {
                    cnt_gps_ts++;
                }
            }

            if(rc < 0)
            {
                return;
            }
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_driver::LidarDriver)