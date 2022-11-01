std_msgs/Header header
int32   gpsweek         # 自1980-1-6至当前的星期数（格林尼治时间）
float64 gpstime         # 自本周日0:00:00至当前的秒数（格林尼治时间）
float32 heading         # 偏航角（0～359.99）
float32 pitch           # 俯仰角（-90～90）
float32 roll            # 横滚角（-180～180）
float64 latitude        # 纬度（-90～90）
float64 longitude       # 经度（-180～180）
float64 altitude        # 高度，单位（米）
float32 ve              # 东向速度，单位（米/秒）
float32 vn              # 北向速度，单位（米/秒）
float32 vu              # 天向速度，单位（米/秒）
float32 baseline        # 基线长度，单位（米）
int16   nsv1            # 天线1卫星数
int16   nsv2            # 天线2卫星数
uint8   status          # 系统状态： 
                        # 低半字节ASCII码： 
                        # 0：初始化 1：粗对准 2：精对准 3：GPS定位 4：GPS定向 5：RTK定位 
                        # 6：DMI组合 7：DMI标定 8：纯惯性 9：零速校正 B：RTK定向
                        # C：进入导航初始化 F：惯导数据异常 
                        # 高半字节ASCII码：
                        # 0：初始化 4：RTK固定解 5：RTK浮点解
string low_status_name
string high_status_name

