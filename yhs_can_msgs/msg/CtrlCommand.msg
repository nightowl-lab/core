std_msgs/Header header

# 禁用档位
uint8 GEAR_DISABLE=0
# P档位
uint8 GEAR_P=1
# R档位
uint8 GEAR_R=2
# N档位
uint8 GEAR_N=3
# D档位
uint8 GEAR_D=4

# 档位,参考GEAR_开头常量
uint8 gear

# 速度,单位m/s
float32 velocity

# 转向角,单位弧度
float32 steering

# 制动力度,范围0-100
uint8 brake
