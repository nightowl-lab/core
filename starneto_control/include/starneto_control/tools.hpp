#ifndef STARNETO_CONTROL_TOOL_H_
#define STARNETO_CONTROL_TOOL_H_

/* 度分格式转为度 */
#define DEGREE(degree_minute) floor((degree_minute) / 100.0)
#define MINUTE(degree_minute) (degree_minute - (DEGREE(degree_minute) * 100.0))
#define DEGMIN_TO_DEG(degree_minute) DEGREE(degree_minute) + MINUTE(degree_minute) / 60.0

/**
 *分割字符串工具宏
 *@param token		分割之后的结果
 *@param result	   	裁剪字符串
 *@param delimiter 	分割符
 */
#define SPLIT_STRING(token, result, delimiter) {\
	token = strtok(NULL, delimiter);\
	result = token;\
	length = strlen(result);\
	result[length] = '\0';\
}

/* GNSS偏航角限制 */
#define GNSS_HEADING_LIMIT(gnss_heading) if((gnss_heading) > 180.0)\
		{(gnss_heading) = 360 - (gnss_heading);} \
	else\
		{(gnss_heading) = -(gnss_heading);}

/**
 *检验校验码是否为十六进制 
 *@param check_char 检验字符
 *@param str		错误提示信息
 */
#define CHECK_HEX(check_char, str) \
    if(!(((check_char) >= '0' && (check_char) <= '9') || ((check_char) >= 'A' && (check_char) <= 'F'))){\
        RCLCPP_ERROR(this->get_logger(), "ERROR: %s: Can't pass hexadecimal test!\n", str);\
		return;\
	}

/**
 *获取校验码
 *@param buffer		 	校验码缓存(char*)
 *@param cs_received 	校验码存储变量(int)
 */
#define RECEIVED_CHECKCODE(buffer, cs_received){\
	char str[3];\
	str[0] = buffer[0];\
	str[1] = buffer[1];\
	str[2] = '\0';\
	sscanf(str, "%x", &cs_received);\
}

#endif