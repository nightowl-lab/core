#ifndef YHS_CAN_CONTROL__TOOL_H__
#define YHS_CAN_CONTROL__TOOL_H__

/**
 * 计算数据包BCC值
 * @param data 数据包
 */
#define CALC_BCC(data) ((data)[0] ^ (data)[1] ^ (data)[2] ^ (data)[3] ^ (data)[4] ^ (data)[5] ^ (data)[6])

/**
 * 检查数据包BCC校验
 * @param data 数据包
 */
#define CHECK_BCC(data) if (CALC_BCC(data) != (data)[7]) {\
        RCLCPP_WARN(this->get_logger(), "BCC validation failed!");\
        return;\
    }

/**
 * 检查数据包心跳计数器
 * @param id CAN ID
 * @param data 数据包
 * @param counter 计数变量
 */
#define CHECK_ALIVE_COUNTER(id, data, counter) if ((data)[6] >> 4 != (counter)[(id)]) {\
        RCLCPP_WARN(this->get_logger(), "Detected package lost, check can cable is stable. %X %d %d", id, (data)[6] >> 4, counter[id]);\
    } \
    (counter)[(id)] = (((data)[6] >> 4) + 1) % 16;

/**
 * 给消息添加头部
 * @param 消息
 * @param frameID 头部frame_id
 * @param time 时间
 */
#define ADD_HEADER_TO_MESSAGE(message, frameID, time) {(message).header.frame_id = frameID;(message).header.stamp = time;}

/**
 * 角度转弧度
 * @param degree 角度
 * @return 弧度
 */
#define DEGREE_TO_RAD(degree) ((degree) / 180.0f * M_PI)

/**
 * 弧度转角度
 * @param rad 弧度
 * @return 角度
 */
#define RAD_TO_DEGREE(rad) ((rad) / M_PI * 180.0f)

/**
 * 区间限制
 * @param min 最小值
 * @param max 最大值
 * @param val 输入值
 * @return 输出值
 * 
 */
#define INTERVAL_LIMIT(min, max, val) ((val) > (max) ? (max) : (val) < (min) ? (min) : (val))

/**
 * 检查是否在区间内
 * @param min 最小值
 * @param max 最大值
 * @param val 输入值
 * @return 是否在区间内
 * 
 */
#define CHECK_IN_INTERVAL(min, max, val) ((val) > (max) ? false : (val) < (min) ? false : true)

#endif