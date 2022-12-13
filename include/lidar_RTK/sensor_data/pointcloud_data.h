#ifndef LIDAR_RTK_SENSOR_DATA_POINTCLOUD_DATA_H_
#define LIDAR_RTK_SENSOR_DATA_POINTCLOUD_DATA_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointcloudData {
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;
public:
    PointcloudData() :cloud_ptr(new CLOUD()) {}
public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};

#endif
