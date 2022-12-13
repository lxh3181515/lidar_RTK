#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_RTK/subscriber/pointcloud_subscriber.h"
#include "lidar_RTK/subscriber/imu_subscriber.h"
#include "lidar_RTK/subscriber/gnss_subscriber.h"
#include "lidar_RTK/subscriber/velocity_subscriber.h"

#include "lidar_RTK/publisher/pointcloud_publisher.h"
#include "lidar_RTK/publisher/odometry_publisher.h"

#include "lidar_RTK/tf_listener/tf_listener.h"

#include "lidar_RTK/models/scan_adjust/distortion_adjust.h"

class DataPretreatFlow {
public:
    DataPretreatFlow(ros::NodeHandle& nh);
    bool run();

private:
    bool readData();
    bool initGNSS();
    bool initCalibration();
    bool hasData();
    bool isValidData();
    bool transformData();
    bool publishData();

    void printDataBuff();

private:
    // Subcriber
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;

    // Publisher
    std::shared_ptr<PointcloudPublisher> pointcloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> odometry_pub_ptr_;

    // TF
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;

    // Distortion adjust
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    // Data buff
    std::deque<PointcloudData> pointcloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    // Current data
    PointcloudData current_pointcloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;
    Eigen::Matrix4f current_gnss_pose_;
    Eigen::Matrix4f lidar_to_imu_;
};

#endif
