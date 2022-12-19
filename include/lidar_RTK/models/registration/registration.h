#ifndef LIDAR_RTK_MODELS_REGISTRATION_REGISTRATION_H_
#define LIDAR_RTK_MODELS_REGISTRATION_REGISTRATION_H_

#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "Eigen/Dense"

class Registration {
public:
    virtual ~Registration() = default;

    virtual bool setInputTarget(const PointcloudData::CLOUD_PTR &pointcloud_input) = 0;
    virtual bool scanMatch(const PointcloudData::CLOUD_PTR &cloud_input_ptr,
                           const Eigen::Matrix4f &predict_pose, 
                           PointcloudData::CLOUD_PTR &result_cloud_ptr,
                           Eigen::Matrix4f &result_pose) = 0;
};

#endif
