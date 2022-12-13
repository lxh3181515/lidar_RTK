#include "lidar_RTK/sensor_data/imu_data.h"

#include <ros/ros.h>

Eigen::Matrix3f IMUData::orientation2Matrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    return q.matrix().cast<float>();
}


bool IMUData::syncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
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
            ROS_WARN("IMU data frames time error.");
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2) 
        return false;

    // Linear interpolation
    IMUData front_data = UnsyncedData.front();
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;
    float front_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    float back_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    
    synced_data.time = sync_time;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    synced_data.orientation.normalize();
    SyncedData.push_back(synced_data);

    return true;
}
