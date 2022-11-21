#ifndef NIGHTOWL_REMOTE_CONTROLLER__TOOL_H__
#define NIGHTOWL_REMOTE_CONTROLLER__TOOL_H__

#include <vector>
#include <stdint.h>

#define TOOL_RAD_TO_DEG 57.29577951

namespace nightowl_remote_controller
{

class Tool
{
  public:
    /**
     * CRC8计算
     * 
     * @param data 数据
     * @return uint8_t CRC8校验码
     */
    static uint8_t crc8(std::vector<std::byte> data);
};

}  // namespace nightowl_remote_controller

#endif