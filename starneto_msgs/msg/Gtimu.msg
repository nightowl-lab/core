std_msgs/Header header
int32   gpsweek     # 自1980-1-6至当前的星期数(接收机时间)
float64 gpstime     # 星期内的秒数（GPS接收机时间）
float64 gx          # 陀螺仪X轴角速率，单位（度/秒）
float64 gy          # 陀螺仪Y轴角速率，单位（度/秒）
float64 gz          # 陀螺仪Z轴角速率，单位（度/秒）
float64 ax          # 加速度计X轴加速度，单位为（g）
float64 ay          # 加速度计Y轴加速度，单位为（g）
float64 az          # 加速度计Z轴加速度，单位为（g）
float32 tpr         # 温度，单位为摄氏度