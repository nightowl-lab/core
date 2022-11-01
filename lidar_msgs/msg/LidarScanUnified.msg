#Lidar C16 scan packets

#Standard ros message header
std_msgs/Header header

#time to the top of hour
uint64 basetime

#vector of raw packets
LidarPacket[] packets
