#include "lidar_RTK/sensor_data/velocity_data.h"

#include <ros/ros.h>

bool VelocityData::syncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time) {
    // Find two adjacent frames at sync_time
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        // The interval between two frames is too long
        if (sync_time - UnsyncedData.front().time > 0.2 || UnsyncedData.at(1).time - sync_time > 0.2) {
            ROS_WARN("Velo data frames time error.");
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2) 
        return false;

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    VelocityData synced_data;
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;
    synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    SyncedData.push_back(synced_data);

    return true;
}


void VelocityData::transformCoordinate(Eigen::Matrix4f transform_matrixf) {
    Eigen::Matrix4d transform_matrixd = transform_matrixf.cast<double>();
    Eigen::Matrix3d rotation = transform_matrixd.block<3, 3>(0, 0);
    Eigen::Vector3d translation(transform_matrixd(0, 3), transform_matrixd(1, 3), transform_matrixd(2, 3));
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);

    w = rotation * w;
    v = rotation * v + w.cross(translation);

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}
