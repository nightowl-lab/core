#include "starneto_configure/starneto_configure_node.hpp"

namespace starneto_configure
{

static void abort_handler(int sig)
{
    abort();
}

static char* rl_gets()
{
	static char* line_read = NULL;

	if(line_read)
	{
		free(line_read);
		line_read = NULL;
	}

	line_read = readline("(INS_CONFIG) ");

	if(line_read && *line_read)
	{
		add_history(line_read);
	}

	return line_read;
}

static void configure_menu()
{
	printf("\nStarneto INS Configuration Node\n");
	printf("\33[1;31m\33[1;43m*****************************************************\n");
	printf("Don't Forget to Save Your Settings And Reboot the INS\n");
	printf("*****************************************************\33[0m\n");
	printf("%d.  Save the setting\n", save_config);
	printf("%d.  Restore the default setting\n", restore_config);
	printf("%d.  Get Sysinfo setting\n", sysinfo_config);
	printf("%d.  Configure Serial\n", serial_config);
	printf("%d.  Configure Network Interface\n", network_config);
	printf("%d.  Configure Output Protocol\n", protocol_config);
	printf("%d.  Configure GNSS Transparent Transmission Protocol\n", through_config);
	printf("%d.  Configure Leverarm Preparation\n", leverarm_config);
	printf("%d.  Configure GNSS Heading Compensation\n", heading_config);
	printf("%d. Configure Navigation Mode\n", navigation_config);
	printf("%d. Configure Coordinate Axis\n", axis_config);
	printf("%d. Configure USB as Storage\n", usb_store_config);
	printf("%d. Quit the starneto configure node\n", quit);
	printf("\n");
}

static char* serial_settings()
{

	char* port_name = NULL;
	char* baudrate = NULL;
	char* parity = NULL;
	char* databit = NULL;
	char* stopbit = NULL;
	char* type = NULL;
	char choose;

	printf("\nINS Serial Port Configuration\n");
	printf("1. Configure the Serial Port\n");
	printf("2. Get the settings of Serial Port\n");
	printf("Choose: ");
	choose = getchar();

	if(choose == '1')
	{
		port_name = readline("(Serial Port [com0, com1]) ");
		baudrate = readline("(Baud Rate [460800 230400 115200 57600 38400 19200 9600]) ");
		parity = readline("(Parity [none]) ");
		databit = readline("(Data Bit [8]) ");
		stopbit = readline("(Stop Bit [1]) ");
		type = readline("(Type [log, rtk]) ");

		sprintf(cmd, CONFIG_HEADER "set,%s,%s,%s,%s,%s,%s" CONFIG_TAIL, port_name, baudrate, parity, databit, stopbit, type);
	}
	else if(choose == '2')
	{
		strcpy(cmd, SERIAL_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		cmd = NULL;
	}

	return cmd;
}


static char* network_settings()
{
	char* local_ip = NULL;
	char* local_mask = NULL;
	char* local_gateway = NULL;
	char* server_ip = NULL;
	char* server_port = NULL;
	char* net_user = NULL;
	char* net_password = NULL;
	char* mountpoint = NULL;
	char choose;
	char network_choose;

	printf("\nINS Network Interface Configuration\n");
	printf("1. Configure the Network Interface\n");
	printf("2. Get the settings of Network Interface\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		printf("1. Configure the Local Address\n");
		printf("2. Configure the Local Mask\n");
		printf("3. Configure the Local Gate\n");
		printf("4. Configure the Server IP and Port\n");
		printf("5. Configure the User and Password \n");
		printf("6. Configure the Mount Point\n");
		printf("Choose: ");
		network_choose = getchar();

		if(network_choose == '1'){
			local_ip = readline("(Local IP [192,168,1,9]) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s" CONFIG_TAIL, "localip", local_ip);
		}
		else if(network_choose == '2'){
			local_mask = readline("(Local Mask [255,255,255,0]) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s" CONFIG_TAIL, "localmask", local_mask);
		}
		else if(network_choose == '3'){
			local_gateway = readline("(Local Gateway [192,168,1,1]) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s" CONFIG_TAIL, "localgate", local_gateway);
		}
		else if(network_choose == '4'){
			server_ip = readline("(Server IP) ");
			server_port = readline("(Server Port) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s,%s" CONFIG_TAIL, "netipport", server_ip, server_port);
		}
		else if(network_choose == '5'){
			net_user = readline("(User) ");
			net_password = readline("(Password) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s:%s" CONFIG_TAIL, "netuser", net_user, net_password);
		}
		else if(network_choose == '6'){
			mountpoint = readline("(Mount Point [AUTO]) ");
			sprintf(cmd, CONFIG_HEADER "set,%s,%s" CONFIG_TAIL, "mountpoint", mountpoint);
		}

	}
	else if(choose == '2')
	{
		strcpy(cmd, NETWORK_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

static char* protocol_settings()
{
	char* port_name = NULL;
	char* protocol_name = NULL;
	char* hz = NULL;
	char choose;

	printf("\nINS Protocol Configuration\n");
	printf("1. Configure the Protocol\n");
	printf("2. Get the settings of Protocol\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		port_name = readline("(Port [com0, usb0, net0]) ");

		protocol_name = readline("(Protocol [gpfpd, gtimu, gphpd, gpgga, gprmc]) ");

		hz = readline("(Message Frequency [0.01, 0.05, 0.1, 0.2, 0.5, 1, null]) ");

		sprintf(cmd, CONFIG_HEADER "output,%s,%s,%s" CONFIG_TAIL, port_name, protocol_name, hz);

	}
	else if(choose == '2')
	{
		strcpy(cmd, PROTOCOL_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

static char* through_settings()
{
	char* port_name = NULL;
	char* msg_name = NULL;
	char* hz = NULL;

	port_name = readline("(Port Name [com0, com1]) ");
	msg_name = readline("(Message Name [bestposb, rangeb, gpsephemb, gloephemerisb, bdsephemerisb, headingb]) ");
	hz = readline("Message Frenquency (0.2, 1, null) ");

	sprintf(cmd, CONFIG_HEADER "through,%s,%s,%s" CONFIG_TAIL, port_name, msg_name, hz);

	return cmd;
}

static char* leverarm_settings()
{
	char* mode = NULL;
	char* x_offset = NULL;
	char* y_offset = NULL;
	char* z_offset = NULL;
	char choose;

	printf("\nINS Leverarm Configuration\n");
	printf("1. Configure the Leverarm\n");
	printf("2. Get the settings of Leverarm\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		mode = readline("(Mode [gnss, point(reserve)]) ");
		x_offset = readline("(X Offset [1.0 (m)]) ");
		y_offset = readline("(Y Offset [1.0 (m)]) ");
		z_offset = readline("(Z Offset [1.0 (m)]) ");

		sprintf(cmd, CONFIG_HEADER "set,leverarm,%s,%s,%s,%s" CONFIG_TAIL, mode, x_offset, y_offset, z_offset);

	}
	else if(choose == '2')
	{
		strcpy(cmd, LEVERARM_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

static char* heading_settings()
{
	char* x_offset = NULL;
	char choose;

	printf("\nINS Heading Compensate Configuration\n");
	printf("1. Configure the Heading Compensate\n");
	printf("2. Get the settings of Heading Compensate\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		x_offset = readline("(X Offset [90 (degree)]) ");

		sprintf(cmd, CONFIG_HEADER "set,headoffset,%s" CONFIG_TAIL, x_offset);

	}
	else if(choose == '2')
	{
		strcpy(cmd, HEADING_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

static char* nav_settings()
{
	char* mode = NULL;
	char* data = NULL;
	char choose;

	printf("\nINS Navigation Mode Configuration\n");
	printf("1. Configure the Navigation Mode\n");
	printf("2. Get the settings of Navigation Mode\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		printf("finealign(on, off)\ngnss(none, single, double)\n"
				"carmode(on, off)\nzupt(on, off)\ncorsealign(on, off)\n"
				"dynamicalign(on, off)\nfirmwareindex(1, 0[default])\n");
		mode = readline("(NAV Mode [finealign, gnss, carmode, zupt, corsealign, dynamicalign, firmwareindex]) ");
		data = readline("(Data [on, off, none, 0, 1]) ");

		sprintf(cmd, CONFIG_HEADER "set,navmode,%s,%s" CONFIG_TAIL, mode, data);

	}
	else if(choose == '2')
	{
		strcpy(cmd, NAV_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

static char* axis_settings()
{
	char* x = NULL;
	char* y = NULL;
	char* z = NULL;
	char choose;

	printf("\nINS Coordinate Axis Configuration\n");
	printf("1. Configure the Coodinate Axis\n");
	printf("2. Get the settings of Coordinate Axis\n");
	printf("Choose: ");
	choose = getchar();
	FLUSH_BUFFER;

	if(choose == '1')
	{
		x = readline("(X Axis [x,-x,y,-y,z,-z]) ");

		y = readline("(Y Axis [x,-x,y,-y,z,-z]) ");

		z = readline("(Z Axis [x,-x,y,-y,z,-z]) ");

		sprintf(cmd, CONFIG_HEADER "set,coordinate,%s,%s,%s" CONFIG_TAIL, x, y, z);

	}
	else if(choose == '2')
	{
		strcpy(cmd, AXIS_CONFIG);
	}
	else
	{
		printf("Error: Unknow Choice!!!\n");
		
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
	}

	return cmd;
}

StarnetoConfigureNode::StarnetoConfigureNode(const rclcpp::NodeOptions & nodeOptions)
    : Node("starneto_configure_node", nodeOptions)
{
    signal(SIGINT, abort_handler);
    RCLCPP_INFO(this->get_logger(), "Loading parameters");
    this->configure_server_address_     = this->declare_parameter("configure_server_address", "192.168.1.9");
    this->server_port_                  = this->declare_parameter("udp_server_port", 7001);
    this->configure_client_address_     = this->declare_parameter("configure_client_address", "192.168.1.19");
    this->client_port_                  = this->declare_parameter("udp_client_port", 8001);

    if(!this->socketInit())
    {
        rclcpp::shutdown();
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "\033[1;32m<--Starneto_configure_node started.-->\033[0m");
    this->sendCmd();

}

void StarnetoConfigureNode::sendCmd(void)
{
    int count = 0;
    int socket_len = 0;
	int choose = 0;
	char* str = NULL;
	char* token = NULL;

    socket_len = sizeof(this->client_socket_addr_);

    while (str = rl_gets())
    {
		cmd = (char *)malloc(sizeof(char) * 100);
		token = strtok(str, ".");
		choose = atoi(token);

        if(!strcmp(token, "exit"))
        {
            rclcpp::shutdown();
            break;
        }
		else if(!strcmp(token, "help"))
		{
			configure_menu();
			continue;
		}

		switch (choose)
		{
		case save_config:
			strcpy(cmd, SAVE_CONFIG);
			printf("%s\n", cmd);
			break;
		case restore_config:
			strcpy(cmd, RESTORE_CONFIG);
			printf("%s\n", cmd);
			break;
		case sysinfo_config:
			strcpy(cmd, SYSINFO_CONFIG);
			printf("%s\n", cmd);
			break;
		case serial_config:
			cmd = serial_settings();
			//FLUSH_BUFFER;
			printf("%s\n", cmd);
			break;
		case network_config:
			cmd = network_settings();
			//FLUSH_BUFFER;
			printf("%s\n", cmd);
			break;
		case protocol_config:
			cmd = protocol_settings();
			printf("%s\n", cmd);
			break;
		case through_config:
			cmd = through_settings();
			printf("%s\n", cmd);
			break;
		case leverarm_config:
			cmd = leverarm_settings();
			printf("%s\n", cmd);
			break;
		case heading_config:
			cmd = heading_settings();
			printf("%s\n", cmd);
			break;
		case navigation_config:
			cmd = nav_settings();
			printf("%s\n", cmd);
			break;
		case axis_config:
			cmd = axis_settings();
			printf("%s\n", cmd);
			break;
		case usb_store_config:
			strcpy(cmd, USB_CONFIG);
			printf("%s\n", cmd);
			break;
		case quit:
			exit(0);
			break;
		default:
			printf("Error: Unknow Choice!!!\n");
			break;
		}

        sendto(this->server_fd_, cmd, strlen(cmd), 0, (struct sockaddr*)&this->client_socket_addr_, socket_len);

        sleep(1);

        do
        {
            memset(this->buffer_, 0, BUFFER_SIZE);
            count = recv(this->server_fd_, this->buffer_, BUFFER_SIZE, MSG_DONTWAIT);
			if(this->buffer_[0] == '$')
				RCLCPP_INFO(this->get_logger(), "%s", this->buffer_);
        } while (count > 0);

		choose = 0;
		if(cmd != NULL)
		{
			free(cmd);
			cmd = NULL;
		}
    }
    
    return;
}

bool StarnetoConfigureNode::socketInit(void)
{
    int return_value;
    
    this->server_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    this->client_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if(this->server_fd_ < 0 || this->client_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Can't create the server or client socket!!!");
        return false;
    }

    memset(&this->server_socket_addr_, 0, sizeof(this->server_socket_addr_));
    //INS Server Socket configuration
    this->server_socket_addr_.sin_family         = AF_INET;
    this->server_socket_addr_.sin_addr.s_addr    = inet_addr(this->configure_server_address_.c_str());
    this->server_socket_addr_.sin_port           = htons(this->server_port_);

    memset(&this->client_socket_addr_, 0, sizeof(this->client_socket_addr_));
    //INS Client Socket configuration
    this->client_socket_addr_.sin_family         = AF_INET;
    this->client_socket_addr_.sin_addr.s_addr    = inet_addr(this->configure_client_address_.c_str());
    this->client_socket_addr_.sin_port           = htons(this->client_port_);

    return_value = bind(this->server_fd_, (struct sockaddr*)&this->server_socket_addr_, sizeof(this->server_socket_addr_));

    if(return_value < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Can't bind the server socket!!!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "The configuration of server socket is done.");
    return true;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(starneto_configure::StarnetoConfigureNode)