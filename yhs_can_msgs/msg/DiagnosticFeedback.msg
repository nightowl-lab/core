std_msgs/Header header

# 错误等级,范围0-3,0表示无故障,其他表示故障等级
uint8 fault_level

# 指令模式CAN通讯错误
bool auto_ctrl_command_can_error

# 指令模式IO通讯错误
bool auto_io_command_can_error

# EPS离线
bool eps_offline

# EPS故障
bool eps_fault

# EPS MOS管过温保护
bool eps_mos_over_temperature

# EPS警告
bool eps_warning

# EPS运行故障
bool eps_running_fault

# EPS过流保护
bool eps_over_current

# ECU故障
bool ecu_fault

# EHB离线
bool ehb_offline

# EHB运行模式故障
bool ehb_running_mode_fault

# EHB未使能
bool ehb_disabled

# EHB角度传感器故障
bool ehb_angle_sensor_fault

# ECU过温保护
bool ecu_over_temperature

# ECU电源故障
bool ehb_power_fault

# ECU传感器可信度异常
bool ehb_sensor_reliability_error

# EHB电机故障
bool ehb_motor_fault

# EHB油压传感器故障
bool ehb_oil_pressure_sensor_fault

# EHB油管故障
bool ehb_oil_tubing_fault

# 驱动电机控制器离线
bool driver_mcu_offline

# 驱动电机控制器过温保护
bool driver_mcu_over_temperature

# 驱动电机控制器过压保护
bool driver_mcu_over_voltage

# 驱动电机控制器欠压保护
bool driver_mcu_under_voltage

# 驱动电机控制器短路保护
bool driver_mcu_short_circuit

# 驱动电机控制器急停故障
bool driver_mcu_emergency

# 驱动电机控制器霍尔传感器故障
bool driver_mcu_hall_sensor_fault

# 驱动电机控制器MOS管故障
bool driver_mcu_mos_fault

# 驱动电机控制器失控故障
bool driver_mcu_out_of_control_fault

# BMS离线
bool bms_offline

# 急停故障
bool emergency

# 遥控器关闭警告
bool rc_power_off

# 遥控器离线警告
bool rc_offline

