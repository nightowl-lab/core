#ifndef STARNETO_CONFIGURE_NODE_H__
#define STARNETO_CONFIGURE_NODE_H__

#include <chrono>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <readline/readline.h>
#include <readline/history.h>

using namespace std::chrono_literals;

#define BUFFER_SIZE 1024
#define FLUSH_BUFFER \
    do \
    {\
        char ch; \
        while((ch = getchar()) != '\n' && ch != EOF); \
    }while(0)\

#define CONFIG_HEADER "$cmd,"
#define CONFIG_TAIL "*ff"

#define COMMAND(str) CONFIG_HEADER str CONFIG_TAIL

#define SAVE_CONFIG COMMAND("save,config")
#define RESTORE_CONFIG COMMAND("factory,config")
#define SYSINFO_CONFIG COMMAND("get,sysinfo")
#define SERIAL_CONFIG COMMAND("get,com")
#define NETWORK_CONFIG COMMAND("get,netpara")
#define PROTOCOL_CONFIG COMMAND("get,output")
#define LEVERARM_CONFIG COMMAND("get,leverarm")
#define HEADING_CONFIG COMMAND("get,headoffset")
#define NAV_CONFIG COMMAND("get,navmode")
#define AXIS_CONFIG COMMAND("get,coordinate")
#define USB_CONFIG COMMAND("set,sysmode,usb")

namespace starneto_configure
{

typedef enum
{
    save_config = 1,
    restore_config = 2,
    sysinfo_config = 3,
    serial_config = 4,
    network_config = 5,
    protocol_config = 6,
    through_config = 7,
    leverarm_config = 8,
    heading_config = 9,
    navigation_config = 10,
    axis_config = 11,
    usb_store_config = 12,
    quit = 13
}config_item;

static char* cmd;
static char* rl_gets();
static void configure_menu();
static char* serial_settings();
static char* network_settings();
static char* protocol_settings();
static char* through_settings();
static char* leverarm_settings();
static char* heading_settings();
static char* nav_settings();
static char* axis_settings();
static char* usb_settings();

class StarnetoConfigureNode : public rclcpp::Node
{
public:
    explicit StarnetoConfigureNode(const rclcpp::NodeOptions & nodeOptions);
    ~StarnetoConfigureNode() = default;
    
    bool socketInit(void);
    void sendCmd(void);

private:
    char buffer_[BUFFER_SIZE];
    int server_fd_;
    int server_port_;
    struct sockaddr_in server_socket_addr_;
    std::string configure_server_address_;
    int client_fd_;
    int client_port_;
    struct sockaddr_in client_socket_addr_;
    std::string configure_client_address_;

    /* 主循环 */
    rclcpp::TimerBase::SharedPtr timer_;
};

}
#endif