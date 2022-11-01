#include "starneto_control/starneto_control_node.hpp"

namespace starneto_control
{

static std::string low_status_name[] = { 
    "Initialization",                   //0 初始化
    "Coarse Alignment",                 //1 粗对准
    "Fine Alignment",                   //2 精对准
    "GPS Localization",                 //3 GPS定位
    "GPS Orienteering",                 //4 GPS定向
    "RTK Localization",                 //5 RTK定位
    "DMI Combination",                  //6 DMI组合
    "DMI Calibration",                  //7 DMI标定
    "Pure Inertia",                     //8 纯惯性
    "Zero Speed Correction",            //9 零速校正
    "",                                 //A NULL
    "RTK Orienteering",                 //B RTK定向
    "Enter Navigation Initialization",  //C 进入导航初始化
    "",                                 //D NULL
    "",                                 //E NULL
    "INS Data Exception"                //F 惯导数据异常
};

static std::string high_status_name[] = { 
    "Initialization",       //0  初始化
    "",                     //1  NULL
    "",                     //2  NULL
    "",                     //3  NULL
    "RTK Fixed Solution",   //4  RTK固定解
    "RTK Float Solution"    //5  RTK浮点解
};

static std::string gpgga_status_name[] = { 
    "Initialization",                   //0  初始化
    "Single Point Localization",        //1  单点定位
    "Differential Code",                //2  码差分
    "",                                 //3  NULL
    "Fixed Solution",                   //4  固定解
    "Float Solution"                    //5  浮点解
    "Estimating",                       //6  正在估算
    "Manual Fixed Value",               //7  人工固定值
    "Dead Reckoning Mode",              //8  航位推算模式
    "WAAS Difference",                  //9  WAAS差分
};

static void abort_handler(int sig)
{
    abort();
}

StarnetoControlNode::StarnetoControlNode(const rclcpp::NodeOptions & nodeOptions)
    : Node("starneto_control_node", nodeOptions)
{
    signal(SIGINT, abort_handler);
    RCLCPP_INFO(this->get_logger(), "Loading parameters");
    this->timesync_flag = this->declare_parameter("timesync_flag", false);
    this->node_rate_    = this->declare_parameter("node_rate", 1);
    this->port_         = this->declare_parameter("port", "/dev/ttyACM0");
    this->baudrate_     = this->declare_parameter("baudrate", 115200);
    this->gnss_frame_   = this->declare_parameter<std::string>("gnss_frame", "ins_link");
    this->imu_frame_    = this->declare_parameter<std::string>("imu_frame", "ins_link");

    this->gpfpdPublisher_ = this->create_publisher<starneto_msgs::msg::Gpfpd>("output/topic_gpfpd", 10);
    this->gtimuPublisher_ = this->create_publisher<starneto_msgs::msg::Gtimu>("output/topic_gtimu", 10);
    this->gpggaPublisher_ = this->create_publisher<starneto_msgs::msg::Gpgga>("output/topic_gpgga", 10);

    this->serialInit();
    this->timer_ = this->create_wall_timer(1ms, std::bind(&StarnetoControlNode::timeCallback, this));
    RCLCPP_INFO(this->get_logger(), "\033[1;32m<--Starneto_control_node started-->.\033[0m");
}

StarnetoControlNode::~StarnetoControlNode() { this->serial_.close(); }

void StarnetoControlNode::serialInit(void)
{
    try
    {
        this->serial_.setPort(this->port_);
        this->serial_.setBaudrate(this->baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        this->serial_.setTimeout(timeout);
        this->serial_.open();
        this->serial_.flushInput();
    }
    catch (serial::IOException& exception)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to port ");
        return;
    }

    if(this->serial_.isOpen())
    {
        this->buffer_.clear();
        RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    }
}

/* 计算校验码(异或运算) */
int Calculate_Checkcode(const char* data_buffer)
{
    int length = strlen(data_buffer);
    int cs_calculate = 0;

    for(int i = 1; data_buffer[i] != '*'; i++)
    {
        cs_calculate ^= data_buffer[i];
    }
    
    return cs_calculate;
}

int Calculate_Checkcode(std::string data_buffer)
{
    int cs_calculate = 0;

    for(int i = 1; data_buffer[i] != '*'; i++)
    {
        cs_calculate ^= data_buffer[i];
    }

    return cs_calculate;
}

/********************************
功能：度分格式转换为度GPFPD
********************************/
double DegMin2Deg(double dddmmpmmmm) {
    double ddd = floor(dddmmpmmmm / 100.0);
    double mmpmmmm = dddmmpmmmm - ddd * 100.0;
    return ddd + mmpmmmm / 60.0;
}

void StarnetoControlNode::timeCallback(void)
{
    int buffer_size = 0;
    int buffer_received = 0;

    RCLCPP_DEBUG(this->get_logger(), "Callback function info");

    buffer_size = this->serial_.available();
    
    if(buffer_size > 0)
    {
        this->buffer_.clear();
        buffer_received =  this->serial_.read(this->buffer_, buffer_size);
    
        // std::cout << "Serial Buffer Size: " << buffer_size << std::endl;
        // std::cout << "Received Size: " << buffer_received << std::endl;
        // std::cout << "String Size: " << this->buffer_.size() << std::endl;
        // std::cout << "Buffer: " << this->buffer_;

        if(buffer_received == buffer_size)
            parseData();
    }

    if(this->timesync_flag)
    {
        double gnssTimestamp = this->gnss_.gpsweek * 7 * 24 * 60 * 60 + this->gnss_.gpstime + GPS_TIME_TO_UTC;
        this->gnss_.header.stamp.sec = gnssTimestamp;
        this->gnss_.header.stamp.nanosec = (gnssTimestamp - this->gnss_.header.stamp.sec) * 1e6;


        double imuTimestamp = this->imu_.gpsweek * 7 * 24 * 60 * 60 + this->imu_.gpstime + GPS_TIME_TO_UTC;
        this->imu_.header.stamp.sec = imuTimestamp;
        this->imu_.header.stamp.nanosec = (imuTimestamp - this->imu_.header.stamp.sec) * 1e6;

        /* GPGGA数据没有Week数,使用GNSS计算 */
        int hours = (int)this->gpgga_.gpstime / 10000;
        int minutes = ((int)this->gpgga_.gpstime / 100) % 100;
        double seconds = this->gpgga_.gpstime - hours * 10000 - minutes * 100;
        double gpggaTimestamp = (int)gnssTimestamp - (int)gnssTimestamp % (24 * 60 * 60) + hours * 60 * 60 + minutes * 60 + seconds;
        this->gpgga_.header.stamp.sec = gpggaTimestamp;
        this->gpgga_.header.stamp.nanosec = (gpggaTimestamp - this->gpgga_.header.stamp.sec) * 1e6;
    }


    RCLCPP_DEBUG(this->get_logger(), "Publish the data\n");
    this->gpfpdPublisher_->publish(std::move(this->gnss_));
    this->gtimuPublisher_->publish(std::move(this->imu_));
    this->gpggaPublisher_->publish(std::move(this->gpgga_));
}
void StarnetoControlNode::parseData()
{
    int length = 0;
    char* token = NULL;
    char* string_data = NULL;
    char* status = NULL;
    int cs_received = 0;
    int cs_calculate = 0;
    char* checksum = NULL;
    int pos = 0;
    std::string str;

    pos = this->buffer_.find('\n');
    
    while(pos != this->buffer_.npos)
    {
        str = this->buffer_.substr(0, pos);
        this->buffer_ = this->buffer_.substr(pos + 1, this->buffer_.size());
        pos = this->buffer_.find('\n');

        if(str[0] != '$'){
            str.clear();
            continue;
        }

        for(int i = 0; i < str.size(); i++)
        {
            if(str[i] == ',' || str[i] == '*')
            {
                this->comma_pos.push_back(i);
            }
        }

        for(int i = 0; i < this->comma_pos.size() - 1; i++)
        {
            if(i == 0)
            {
                std::string header = str.substr(i, this->comma_pos[i]);
                this->substrs.push_back(header);
            }

            std::string substr(&str[comma_pos[i] + 1], &str[comma_pos[i + 1]]);

            if(substr.empty())
            {
                this->substrs.push_back("null");
                continue;
            }

            this->substrs.push_back(substr);
        }

        std::string tail = str.substr(comma_pos[comma_pos.size() - 1] + 1);
        this->substrs.push_back(tail);

        //std::cout << "Pos: " << pos - 1 << std::endl;
        //std::cout << "String: " << str << std::endl;

        this->gnss_.header.frame_id = this->gnss_frame_;
        this->gnss_.header.stamp = this->get_clock()->now();
        this->imu_.header.frame_id = this->imu_frame_;
        this->imu_.header.stamp = this->get_clock()->now();
        this->gpgga_.header.frame_id = this->gnss_frame_;
        this->gpgga_.header.stamp = this->get_clock()->now();

        cs_calculate = Calculate_Checkcode(str.c_str());

        
        if(!substrs[0].compare("$GPFPD"))
        {
            RCLCPP_DEBUG(this->get_logger(), "The protocol type is $GPGGA\n");

            this->gnss_.gpsweek     = std::stoi(this->substrs[1]);
            this->gnss_.gpstime     = std::stod(this->substrs[2]);
            this->gnss_.heading     = std::stod(this->substrs[3]);
            this->gnss_.pitch       = std::stod(this->substrs[4]);
            this->gnss_.roll        = std::stod(this->substrs[5]);
            this->gnss_.latitude    = std::stod(this->substrs[6]);
            this->gnss_.longitude   = std::stod(this->substrs[7]);
            this->gnss_.altitude    = std::stod(this->substrs[8]);
            this->gnss_.ve          = std::stod(this->substrs[9]);
            this->gnss_.vn          = std::stod(this->substrs[10]);
            this->gnss_.vu          = std::stod(this->substrs[11]);
            this->gnss_.baseline    = std::stod(this->substrs[12]);
            this->gnss_.nsv1        = std::stoi(this->substrs[13]);
            this->gnss_.nsv2        = std::stoi(this->substrs[14]);

            //Testing the data: 14
            RCLCPP_DEBUG(this->get_logger(), "$GPFPD,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d\n",
                this->gnss_.gpsweek,this->gnss_.gpstime,
                this->gnss_.heading, this->gnss_.pitch, this->gnss_.roll,
                this->gnss_.latitude, this->gnss_.longitude, this->gnss_.altitude,
                this->gnss_.ve, this->gnss_.vn, this->gnss_.vu, this->gnss_.baseline,
                this->gnss_.nsv1, this->gnss_.nsv2);

            std::string status_str = this->substrs[15];
            CHECK_HEX(status_str[0], "GPFPD::STATUS");
            CHECK_HEX(status_str[1], "GPFPD::STATUS");
            RECEIVED_CHECKCODE(status_str, this->gnss_.status);

            uint8_t low_halfbyte = 0;
            uint8_t high_halfbyte = 0;
            low_halfbyte = this->gnss_.status & 0x0F;
            high_halfbyte = (this->gnss_.status & 0xF0) >> 4;

            this->gnss_.low_status_name = low_status_name[low_halfbyte];
            this->gnss_.high_status_name = high_status_name[high_halfbyte];

            std::string checksum_str = this->substrs[this->substrs.size() - 1];

            CHECK_HEX(checksum_str[0], "GPFPD::CHECKSUM");
            CHECK_HEX(checksum_str[1], "GPFPD::CHECKSUM");
            RECEIVED_CHECKCODE(checksum_str, cs_received);

            if(cs_calculate != cs_received)
                RCLCPP_ERROR(this->get_logger(), "GPS: Checksum is not match!!!");
            
            double gpstime_pre = 0;
            int delta_t = this->gnss_.gpstime - gpstime_pre;
            gpstime_pre = this->gnss_.gpstime;

            RCLCPP_DEBUG(this->get_logger(), "GPS DATA OK");

        }
        else if(!substrs[0].compare("$GTIMU"))
        {
            RCLCPP_DEBUG(this->get_logger(), "The protocol type is GTIMU\n");

            this->imu_.gpsweek  = std::stoi(this->substrs[1]);
            this->imu_.gpstime  = std::stod(this->substrs[2]);
            this->imu_.gx       = std::stod(this->substrs[3]);
            this->imu_.gy       = std::stod(this->substrs[4]);
            this->imu_.gz       = std::stod(this->substrs[5]);
            this->imu_.ax       = std::stod(this->substrs[6]);
            this->imu_.ay       = std::stod(this->substrs[7]);
            this->imu_.az       = std::stod(this->substrs[8]);
            this->imu_.tpr      = std::stod(this->substrs[9]);

            //Testing the data: 9
            RCLCPP_DEBUG(this->get_logger(), "$GTIMU,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                this->imu_.gpsweek,this->imu_.gpstime,
                this->imu_.gx, this->imu_.gy, this->imu_.gz,
                this->imu_.ax, this->imu_.ay, this->imu_.az,this->imu_.tpr);

            //SPLIT_STRING(token, checksum, "\n");
            std::string checksum_str = this->substrs[this->substrs.size() - 1];

            CHECK_HEX(checksum_str[0], "GTIMU::CHECKSUM");
            CHECK_HEX(checksum_str[1], "GTIMU::CHECKSUM");
            RECEIVED_CHECKCODE(checksum_str, cs_received);

            if(cs_received != cs_calculate)
                RCLCPP_ERROR(this->get_logger(), "IMU: Checksum is not match!!!");

            RCLCPP_DEBUG(this->get_logger(), "IMU DATA OK");

        }
        else if(!substrs[0].compare("$GPGGA"))
        {
            RCLCPP_DEBUG(this->get_logger(), "The protocol type is GPGGA\n");

            this->gpgga_.gpstime = std::stod(this->substrs[1]);

            this->gpgga_.latitude = std::stod(this->substrs[2]);
            this->gpgga_.latitude = DegMin2Deg(this->gpgga_.latitude); //度分式转度

            this->gpgga_.n = this->substrs[3].c_str();

            this->gpgga_.longitude = std::stod(this->substrs[4]);
            this->gpgga_.longitude = DegMin2Deg(this->gpgga_.longitude);//度分式转度

            this->gpgga_.e = this->substrs[5].c_str();

            std::string status_str = this->substrs[6];
            CHECK_HEX(status_str[0], "GPGGA::STATUS");
            RECEIVED_CHECKCODE(status_str, this->gpgga_.status);

            this->gpgga_.nosv = std::stoi(this->substrs[7]);

            this->gpgga_.hdop = std::stod(this->substrs[8]);

            this->gpgga_.msl = std::stod(this->substrs[9]);
            
            this->gpgga_.m_1 = this->substrs[10];

            this->gpgga_.altref = std::stod(this->substrs[11]);

            this->gpgga_.m_2 = this->substrs[12];

            if(!this->substrs[13].compare("null"))
            {
                this->gpgga_.diff_age = 0;
            }
            else
            {
                this->gpgga_.diff_age = std::stoi(this->substrs[13]);
            }

            if(!this->substrs[14].compare("null"))
            {
                this->gpgga_.diff_station = 0;
            }
            else
            {
                this->gpgga_.diff_station = std::stoi(this->substrs[14]);
            }

            //Testing the data: 13
            RCLCPP_DEBUG(this->get_logger(), "$GPGGA,%d,%lf,%s,%lf,%s,%d,%d,%lf,%s,%lf,%s,%d,%d\n",
                this->gpgga_.gpstime,
                this->gpgga_.latitude, this->gpgga_.n, this->gpgga_.longitude, this->gpgga_.e,
                this->gpgga_.status, this->gpgga_.nosv, this->gpgga_.hdop, this->gpgga_.msl,
                this->gpgga_.m_1, this->gpgga_.altref, this->gpgga_.m_2, this->gpgga_.diff_age,
                this->gpgga_.diff_station);

            this->gpgga_.status_name = gpgga_status_name[this->gpgga_.status];

            std::string checksum_str = this->substrs[this->substrs.size() - 1];
            CHECK_HEX(checksum_str[0], "GPGGA::CHECKSUM");
            CHECK_HEX(checksum_str[1], "GPGGA::CHECKSUM");
            RECEIVED_CHECKCODE(checksum_str, cs_received);
            //std::cout <<  cs_received << std::endl;
            //std::cout <<  cs_calculate << std::endl;

            if(cs_calculate != cs_received)
                RCLCPP_ERROR(this->get_logger(), "GPGGA: Checksum is not match!!!");
            
            double gpstime_pre = 0;
            int delta_t = this->gpgga_.gpstime - gpstime_pre;
            gpstime_pre = this->gpgga_.gpstime;

            RCLCPP_DEBUG(this->get_logger(), "GPGGA DATA OK");
        }

        this->substrs.clear();
        this->comma_pos.clear();
    }

}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(starneto_control::StarnetoControlNode)
