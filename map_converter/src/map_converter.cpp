#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tuple>

#define GZD_ID_SIZE 5
#define PRECISION_METER_SIZE 5
#define MGRS_PRECISION 4

void convertWGS84ToMGRS(double lat, double lon, double height, double & x, double & y, double & z)
{
    double utmNorthing, utmEasting;
    bool utmNorthp;
    int utmZone;
    GeographicLib::UTMUPS::Forward(lat, lon, utmZone, utmNorthp, utmEasting, utmNorthing);
    std::string mgrs;
    GeographicLib::MGRS::Forward(utmZone, utmNorthp, utmEasting, utmNorthing, PRECISION_METER_SIZE + MGRS_PRECISION, mgrs);
    x = std::stod(mgrs.substr(GZD_ID_SIZE, PRECISION_METER_SIZE + MGRS_PRECISION)) * std::pow(10, -MGRS_PRECISION);
    y = std::stod(mgrs.substr(GZD_ID_SIZE + PRECISION_METER_SIZE + MGRS_PRECISION, PRECISION_METER_SIZE + MGRS_PRECISION)) * std::pow(10, -MGRS_PRECISION);
    z = height;
}

int main(int argc, char * argv[])
{
    /* 读取参数*/
    if (argc != 9) {
        printf("ros2 run map_converter map_converter [inputPath] [outputPath] [latitude] [longitude] [height] [roll] [pitch] [yaw]\n");
        printf("inputPath: Absolute path for input PointCloud .pcd file.\n");
        printf("outputPath: Absolute path for output PointCloud .pcd file, note that if a file is already placed on the path, the file will be rewrite.\n");
        printf("latitude, longitude, height, roll, pitch, yaw: Initialization pose of the PointCloud");
        return -1;
    }
    std::string inputPCDPath = argv[1];
    std::string outputPCDPath = argv[2];
    double initPointLat = std::stod(argv[3]);
    double initPointLon = std::stod(argv[4]);
    double initPointHeight = std::stod(argv[5]);
    double roll = std::stod(argv[6]);
    double pitch = std::stod(argv[7]);
    double yaw = std::stod(argv[8]);

    printf("map_converter started.\n");
    /* 计算旋转四元数 */
    tf2::Quaternion tempQuat;
    tempQuat.setRPY(roll, pitch, 2 * M_PI - yaw);
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(tempQuat);
    tf2::Stamped<tf2::Transform> relativePoseTransform;
    relativePoseTransform.setRotation(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w));
    /* 初始化本地笛卡尔坐标系 */
    auto relativeCartesia = GeographicLib::LocalCartesian(initPointLat, initPointLon, initPointHeight);
    /* 读取PCD */
    printf("Reading .pcd file...\n");
    auto source = new pcl::PointCloud<pcl::PointXYZI>();
    if (pcl::io::loadPCDFile(inputPCDPath, *source) < 0) {
        printf("Error loading point cloud\n");
        return -1;
    }
    auto target = new pcl::PointCloud<pcl::PointXYZI>();
    double lastPercent = 0;
    printf("Start converting...\n");
    for (auto iter = source->begin(); iter != source->end(); iter++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = iter->x;
        pose.pose.position.y = iter->y;
        pose.pose.position.z = iter->z;
        tf2::doTransform(pose, pose, tf2::toMsg((relativePoseTransform)));

        double lat, lon, height;
        relativeCartesia.Reverse(-pose.pose.position.y, pose.pose.position.x, pose.pose.position.z, lat, lon, height);

        convertWGS84ToMGRS(lat, lon, height, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        auto point = pcl::PointXYZI();
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.z = pose.pose.position.z;
        point.intensity = iter->intensity;
        target->push_back(point);

        /* 显示进度 */
        double percent = (double)target->size() / (double)source->size();
        if (percent - lastPercent > 0.0001) {
            printf("Converting...%.2f%%\r", percent * 100.0f);
            lastPercent = percent;
        }
    }
    printf("\nWriting to .pcd file...\n");
    const std::string outputPath = outputPCDPath;
    // 成功返回0，失败返回-1
    if (-1 == pcl::io::savePCDFile(outputPath, *target, false)) {
        printf("save pcd file failed");
        return -1;
    } else {
        printf("success!");
    }
}