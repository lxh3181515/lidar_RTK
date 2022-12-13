#include "lidar_RTK/models/scan_adjust/distortion_adjust.h"


void DistortionAdjust::setMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}


bool DistortionAdjust::adjustCloud(PointcloudData::CLOUD_PTR& input_cloud_ptr, PointcloudData::CLOUD_PTR& output_cloud_ptr) {
    PointcloudData::CLOUD_PTR origin_cloud_ptr(new PointcloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr.reset(new PointcloudData::CLOUD());

    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    float orientation_space = 2.0 * M_PI;
    float delete_space = 0.0;

    // Set first point frame.
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    // Adjust point.
    size_t points_size = origin_cloud_ptr->points.size();
    for (size_t index = 1; index < points_size; ++index) {
        float orientation = atan2(origin_cloud_ptr->points[index].y, origin_cloud_ptr->points[index].x);

        if (orientation < 0.0) // [-pi, pi] -> [0, 2 * pi]
            orientation += 2.0 * M_PI;
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[index].x,
                                     origin_cloud_ptr->points[index].y,
                                     origin_cloud_ptr->points[index].z);

        Eigen::Matrix3f current_rotate_matrix = updateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_rotate_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;

        output_cloud_ptr->points.push_back(PointcloudData::POINT(adjusted_point(0), adjusted_point(1), adjusted_point(2)));
    }

    // Reset frame.
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}


Eigen::Matrix3f DistortionAdjust::updateMatrix(float real_time) {
    Eigen::Vector3f real_angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(real_angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(real_angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(real_angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;

    return t_V.matrix();
}
