#ifndef LIDAR_RTK_MODELS_REGISTRATION_REGISTRATION_NDT_H_
#define LIDAR_RTK_MODELS_REGISTRATION_REGISTRATION_NDT_H_

#include "lidar_RTK/models/registration/registration.h"
#include "pcl/registration/ndt.h"

class RegistrationNDT: public Registration {
public:
    using REGISTER = pcl::NormalDistributionsTransform<PointcloudData::POINT, PointcloudData::POINT>;

public:
    RegistrationNDT();

    bool setInputTarget(const PointcloudData::CLOUD_PTR &pointcloud_input) override;
    bool scanMatch( const PointcloudData::CLOUD_PTR &cloud_input_ptr,
                    const Eigen::Matrix4f &predict_pose, 
                    PointcloudData::CLOUD_PTR &result_cloud_ptr,
                    Eigen::Matrix4f &result_pose) override;

private:
    REGISTER::Ptr register_ndt_ptr_;
};

#endif
