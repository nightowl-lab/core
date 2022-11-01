std_msgs/Header header
float64 gpstime      # UTC时间
float64 latitude     # 纬度, 度分格式, 前导位数不足时补零
string  n            # 纬度半球(N 北纬/ S 南纬)
float64 longitude    # 经度, 度分格式, 前导位数不足时补零
string  e            # 经度半球(E 东经/ W 西经)
uint8   status       # GPS状态: 0 = 初始化 1 = 单点定位 2 = 码差分 4 = 固定解 5 = 浮点解 6 = 正在估算 7 = 人工固定值 8 = 航位推算模式 9 = WAAS差分
uint8   nosv         # 正在使用解算位置的卫星数量，前导位数不足时补零
float32 hdop         # HDOP水平精度因子(0.5~99.9)
float32 msl          # 海拔高度
string  m_1          # 单位，M为米
float32 altref       # 地球椭球面相对大地水准面的高度
string  m_2          # 单位，M为米
int16   diff_age     # 差分时间，前导位数不足时补零，从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空
int16   diff_station # 差分站ID号，前导位数不足时补零，如果不是差分定位将为空
string status_name