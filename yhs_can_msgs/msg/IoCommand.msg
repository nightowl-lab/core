std_msgs/Header header

# 关闭转向灯
uint8 TURN_SIGNAL_OFF=0
# 打开左转向灯
uint8 TURN_SIGNAL_LEFT=1
# 打开右转向灯
uint8 TURN_SIGNAL_RIGHT=2

# 是否启用IO
bool enable

# 示廊灯是否开启
bool marker_light

# 转向灯状态,参考TURN_SIGNAL_常量
uint8 turn_signal

# 喇叭是否开启
bool horn

# 充电过程中是否强制上电
bool force_power_on_in_charging