#include "lidar_RTK/models/registration/registration_ndt.h"


RegistrationNDT::RegistrationNDT() {
    register_ndt_ptr_ = pcl::make_shared<REGISTER>();

    register_ndt_ptr_->setResolution(1.0);
    register_ndt_ptr_->setStepSize(0.1);
    register_ndt_ptr_->setTransformationEpsilon(0.01);
    register_ndt_ptr_->setMaximumIterations(30);
}


bool RegistrationNDT::setInputTarget(const PointcloudData::CLOUD_PTR &pointcloud_input) {
    register_ndt_ptr_->setInputTarget(pointcloud_input);
    return true;
}


bool RegistrationNDT::scanMatch(const PointcloudData::CLOUD_PTR &cloud_input_ptr,
                                const Eigen::Matrix4f &predict_pose, 
                                PointcloudData::CLOUD_PTR &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose) {
    register_ndt_ptr_->setInputSource(cloud_input_ptr);
    register_ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = register_ndt_ptr_->getFinalTransformation();
    return true;
}
