#include"sanchi_amov/sanchi_amov.hpp"

static int data_length = 81;
boost::asio::serial_port* serial_port = 0;
static const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
static const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
static uint8_t data_raw[200];
static std::vector<uint8_t> buffer_;
static std::deque<uint8_t> queue_;
static std::string name, frame_id;
static sensor_msgs::msg::Imu msg_imu;
static sensor_msgs::msg::MagneticField msg_mag;
static sensor_msgs::msg::NavSatFix msg_gps;
static int fd_ = -1;
static uint8_t tmp[81];

namespace imu_driver
{

static float d2f_acc(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 16384.0f;
}

static float d2f_gyro(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 32.8f;
}

static float d2f_mag(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 1.0f;
}

static float d2f_euler(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}


static double d2f_latlon(uint8_t a[4])
{
    int64_t high = a[0];
    high = (high << 8) | a[1];

    int64_t low = a[2];
    low = (low << 8) | a[3];
    return (double)((high << 8) | low);
}

static double d2f_gpsvel(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}

static float d2ieee754(uint8_t a[4])
{
    union fnum {
        float f_val;
        uint8_t d_val[4];
    } f;

    memcpy(f.d_val, a, 4);
    return f.f_val;
}


int uart_set(int fd, int baude, int c_flow, int bits, char parity, int stop)
{
    struct termios options;
 
    if(tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }

    cfsetispeed(&options,B115200);
    cfsetospeed(&options,B115200);

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;

    switch(c_flow)
    {
        case 0:
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1:
            options.c_cflag |= CRTSCTS;
            break;
        case 2:
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    switch(parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~INPCK;
            break;

        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;

        case 'o':
        case 'O':
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;

        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;

    tcflush(fd,TCIFLUSH);

    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;

}

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions & options)
    : Node("imu_driver", options)
{
    this->model_    = this->declare_parameter<std::string>("model", std::string("100D2"));
    this->port_     = this->declare_parameter<std::string>("port", std::string("/dev/ttyUSB0"));
    this->baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    this->delay_    = this->declare_parameter<double>("delay", 0.0);
    this->frame_id_ = this->declare_parameter<std::string>("frame_id", std::string("base_link"));


    this->imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
    this->magPublisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag_data", 1);
    this->gpsPublisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_data", 1);
    serial_setup();

    this->timer_ = this->create_wall_timer(10ms, std::bind(&ImuDriverNode::timeCallback, this));
    RCLCPP_INFO(this->get_logger(), "\033[1;32m<--Imu_driver_node started-->.\033[0m");
}

ImuDriverNode::~ImuDriverNode()
{
    ::close(fd_);
}

void ImuDriverNode::timeCallback(void)
{
    parse_data();
}

void ImuDriverNode::serial_setup(void)
{
    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);

    try
    {
        serial_port->open(this->port_);
    }
    catch(boost::system::system_error &error)
    {
        RCLCPP_ERROR(this->get_logger(), "%s: Failed to open port %s with error %s",
                name.c_str(), this->port_.c_str(), error.what());
    }
    
    if(!serial_port->is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "%s: failed to open serial port %s",
                name.c_str(), this->port_.c_str());
    }

    typedef boost::asio::serial_port_base serial;

    serial::baud_rate baud_option(this->baudrate_);
    serial::flow_control flow_control(serial::flow_control::none);
    serial::parity parity(serial::parity::none);
    serial::stop_bits stop_bits(serial::stop_bits::one);

    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);

    const char* path = this->port_.c_str();
    fd_ = open(path, O_RDWR);
    if(fd_ < 0)
    {    
        RCLCPP_ERROR(this->get_logger(), "Port Error!: %s", path);
    }

    if(this->model_ == "100D2")
    {
        write(fd_, stop, 6);
        usleep(1000 * 1000);
        write(fd_, mode, 6);
        usleep(1000 * 1000);
        data_length = 40;
    }
}

void ImuDriverNode::parse_data(void)
{
    RCLCPP_INFO(this->get_logger(), "Streaming Data...");

    while (rclcpp::ok())
    {
        read(fd_, tmp, sizeof(uint8_t) * data_length);
        memcpy(data_raw, tmp, sizeof(uint8_t) * data_length);

        bool found = false;

        for(int i = 0; i < data_length - 1; ++i)
        {
            if(this->model_ == "100D2" && data_raw[i] == 0xA5 && data_raw[i+1] == 0x5A)
            {
                unsigned char* data = data_raw + i;

                uint8_t data_length = data[2];

                uint32_t checksum = 0;
                for(int j = 0; j < data_length - 1  ; ++j)
                {
                    checksum += (uint32_t)data[j+2];
                }

                uint16_t check = checksum % 256;
                uint16_t check_true = data[data_length+1];

                if (check != check_true)
                {
                    printf("check error\n");
		    break;
                }
		
                Eigen::Vector3d ea0(-d2f_euler(data+3) * M_PI / 180.0,      //yaw
                                    d2f_euler(data+7) * M_PI / 180.0,       //pitch
                                    -d2f_euler(data+5) * M_PI / 180.0);      //roll
                // Eigen::Matrix3d rotate_matrix;
                // rotate_matrix << 1,  0,  0,
                //                  0,  0,  1,
                //                  0, -1,  0;
                Eigen::Matrix3d R;

                R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) 
                  * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
                //R =  R * rotate_matrix;
                Eigen::Quaterniond q;
                q = R;
                msg_imu.orientation.w = (double)q.w();
                msg_imu.orientation.x = (double)q.x();
                msg_imu.orientation.y = (double)q.y();
                msg_imu.orientation.z = (double)q.z();

                msg_imu.header.stamp = this->get_clock()->now();
                msg_imu.header.frame_id = this->frame_id_;
                // msg_imu.angular_velocity.x = d2f_gyro(data + 15);
                // msg_imu.angular_velocity.y = d2f_gyro(data + 17);
                msg_imu.angular_velocity.x = d2f_gyro(data + 17) * M_PI / 180.0;
                msg_imu.angular_velocity.y = -d2f_gyro(data + 15) * M_PI / 180.0;
                msg_imu.angular_velocity.z = d2f_gyro(data + 19) * M_PI / 180.0;
                // msg_imu.linear_acceleration.x = d2f_acc(data + 9) * 9.81;
                // msg_imu.linear_acceleration.y = d2f_acc(data + 11) * 9.81;
                msg_imu.linear_acceleration.x = d2f_acc(data + 11) * 9.81;
                msg_imu.linear_acceleration.y = -d2f_acc(data + 9) * 9.81;
                msg_imu.linear_acceleration.z = d2f_acc(data + 13) * 9.81;
                this->imuPublisher_->publish(msg_imu);

                msg_mag.magnetic_field.x = d2f_mag(data + 21) / 10000.0f;
                msg_mag.magnetic_field.y = d2f_mag(data + 23) / 10000.0f;
                msg_mag.magnetic_field.z = d2f_mag(data + 25) / 10000.0f;
                msg_mag.header.stamp = msg_imu.header.stamp;
                msg_mag.header.frame_id = msg_imu.header.frame_id;
	    	    this->magPublisher_->publish(msg_mag);
                found = true;
            }
        }
    }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_driver::ImuDriverNode)
