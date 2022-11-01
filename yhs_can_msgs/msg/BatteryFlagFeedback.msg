std_msgs/Header header

# 剩余电量百分比,范围0-100
uint8 percent

# 单体过压保护标志位
bool single_over_voltage

# 单体欠压保护标志位
bool single_under_voltage

# 整组过压保护标志位
bool over_voltage

# 整组欠压保护标志位
bool under_voltage

# 充电过程过温保护标志位
bool charge_over_temperature

# 充电过程低温保护标志位
bool charge_under_temperature

# 充电过程过流保护标志位
bool charge_over_current

# 放电过程过温保护标志位
bool discharge_over_temperature

# 放电过程低温保护标志位
bool discharge_under_temperature

# 放电过程过流保护标志位
bool discharge_over_current

# 短路保护标志位
bool short_circuit

# 检测IC错误标志位
bool ic_error

# mos管锁定标志位
bool mos_locked

# 是否正在充电标志位
bool charging

# 当前电池最高温度,单位摄氏度
float32 max_temperature_in_battry

# 当前电池最低温度,单位摄氏度
float32 min_temperature_in_battry
