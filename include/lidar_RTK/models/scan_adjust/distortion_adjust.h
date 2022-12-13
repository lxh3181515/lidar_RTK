#ifndef LIDAR_RTK_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#define LIDAR_RTK_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_

#include "lidar_RTK/sensor_data/velocity_data.h"
#include "lidar_RTK/sensor_data/pointcloud_data.h"

#include <Eigen/Dense>
#include "pcl/common/transforms.h"

class DistortionAdjust {
public:
    void setMotionInfo(float scan_period, VelocityData velocity_data);
    bool adjustCloud(PointcloudData::CLOUD_PTR& input_cloud_ptr, PointcloudData::CLOUD_PTR& output_cloud_ptr);

private:
    inline Eigen::Matrix3f updateMatrix(float real_time);

private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};

#endif
