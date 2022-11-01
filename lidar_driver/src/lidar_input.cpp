#include "lidar_driver/lidar_input.hpp"

extern volatile sig_atomic_t flag;

namespace lidar_driver
{
    static const size_t packet_size = sizeof(lidar_msgs::msg::LidarPacket().data);

    Input::Input(rclcpp::Node::SharedPtr node_ptr, uint16_t port)
        : node_ptr_(node_ptr), port_(port)
    {
        this->npkt_update_flag_ = false;
        this->current_rpm_ = 0;
        this->return_mode_ = 1;
        
        if(!this->node_ptr_->has_parameter("device_ip"))
            this->device_ip_str_ = this->node_ptr_->declare_parameter("device_ip", std::string(""));

        if(!this->device_ip_str_.empty())
        {
            RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "Only accepting packets from IP address: " << this->device_ip_str_);
        }
    }

    Input::~Input() {}

    int Input::getRpm(void)
    {
        return current_rpm_;
    }

    int Input::getReturnMode(void)
    {
        return this->return_mode_;
    }

    bool Input::getUpdateFlag(void)
    {
        return this->npkt_update_flag_;
    }

    void Input::clearUpdateFlag(void)
    {
        this->npkt_update_flag_ = false;
    }

    InputSocket::InputSocket(rclcpp::Node::SharedPtr node_ptr, uint16_t port)
        : Input(node_ptr, port)
    {
        this->socket_fd_ = -1;

        if(!this->device_ip_str_.empty())
        {
            inet_aton(this->device_ip_str_.c_str(), &this->device_ip_);
        }

        RCLCPP_INFO_STREAM(this->node_ptr_->get_logger(), "Opening UDP socket: port " << port);

        this->socket_fd_ = socket(PF_INET, SOCK_DGRAM, 0);

        if(this->socket_fd_ == -1)
        {
            perror("socket");
            return;
        }

        int opt = 1;
        if(setsockopt(this->socket_fd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
        {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;
        memset(&my_addr, 0, sizeof(my_addr));
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(port);
        my_addr.sin_addr.s_addr = INADDR_ANY;

        if(bind(this->socket_fd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
        {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }

        if(fcntl(this->socket_fd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
        {
            perror("non-block");
            return;
        }
    }

    InputSocket::~InputSocket(void)
    {
        (void)close(this->socket_fd_);
    }

    int InputSocket::getPacket(lidar_msgs::msg::LidarPacket* pkt, const double time_offset)
    {
        double time_start = this->node_ptr_->get_clock()->now().seconds();

        struct pollfd fds[1];
        fds[0].fd = this->socket_fd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 1200; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);

        while(flag == 1)
        {
            do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);

                if(retval < 0)
                {
                    if(errno != EINTR)
                    {
                        RCLCPP_ERROR(this->node_ptr_->get_logger(), "poll() error: %s", strerror(errno));
                    }   
                    return 1;
                }

                if(retval == 0)
                {
                    time_t curTime = time(NULL);
                    struct tm *curTm = localtime(&curTime);
                    char bufTime[30] = {0};
                    sprintf(bufTime,"%d-%d-%d %d:%d:%d", curTm->tm_year+1900, curTm->tm_mon+1,
                            curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);

                    //rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(), *this->node_ptr_->get_clock(), 2, "%s  lidar poll() timeout", bufTime);

                    return 1;
                }

                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
                {
                    RCLCPP_ERROR(this->node_ptr_->get_logger(), "poll() reports lidar error");
                    return 1;
                }

            } while ((fds[0].revents & POLLIN) == 0);
                    
            ssize_t nbytes = recvfrom(this->socket_fd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    RCLCPP_ERROR(this->node_ptr_->get_logger(), "recvfail");
                    return 1;
                }
            }
            else if ((size_t)nbytes == packet_size)
            {
                if (this->device_ip_str_ != "" && sender_address.sin_addr.s_addr != this->device_ip_.s_addr)
                    continue;
                else
                    break;  // done
            }

            RCLCPP_DEBUG_STREAM(this->node_ptr_->get_logger(), "incomplete lidar packet read: " << nbytes << " bytes");
        }

        if (flag == 0)
        {
            abort();
        }

        if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
        {   
            //difop
            int rpm = (pkt->data[8] << 8) | pkt->data[9];

            int mode = 1;

            if (current_rpm_ != rpm || return_mode_ != mode)
            {
                current_rpm_ = rpm;
                return_mode_ = mode;

                npkt_update_flag_ = true;
            }
        }
        
        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred. Add the time offset.
        double time_end = this->node_ptr_->get_clock()->now().seconds();
        pkt->stamp = rclcpp::Time((time_end + time_start) / 2.0 + time_offset);

        return 0;
    }
}