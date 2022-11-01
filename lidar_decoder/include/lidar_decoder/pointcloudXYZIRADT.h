#ifndef __POINTCLOUDXYZIRADT_H
#define __POINTCLOUDXYZIRADT_H

#include <pcl/point_cloud.h>

#include <lidar_decoder/datacontainerbase.h>
#include <lidar_decoder/point_types.h>

namespace lslidar_pointcloud
{
class PointcloudXYZIRADT : public lslidar_rawdata::DataContainerBase
{
public:
  pcl::PointCloud<lslidar_pointcloud::PointXYZIRADT>::Ptr pc;

  PointcloudXYZIRADT() : pc(new pcl::PointCloud<lslidar_pointcloud::PointXYZIRADT>) {}

  virtual void addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & return_type, const uint16_t & ring, const uint16_t & azimuth,
    const float & distance, const float & intensity,
    const double & time_stamp) override;
};
}  // namespace lslidar_pointcloud
#endif