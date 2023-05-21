#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "lidar_RTK/subscriber/gnss_subscriber.h"
#include "lidar_RTK/subscriber/pointcloud_subscriber.h"
#include "lidar_RTK/publisher/pointcloud_publisher.h"
#include "lidar_RTK/publisher/odometry_publisher.h"

class GNSSPretreatFlow {
public:
    GNSSPretreatFlow(ros::NodeHandle& nh);
    bool run();

private:
    bool readData();
    bool initGNSS();
    bool hasData();
    bool isValidData();
    bool transformData();
    bool publishData();


private:
    // Subcriber
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;

    // Publisher
    std::shared_ptr<PointcloudPublisher> pointcloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> odometry_pub_ptr_;

    // Data buff
    std::deque<PointcloudData> pointcloud_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    // Current data
    PointcloudData current_cloud_data_;
    GNSSData current_gnss_data_;
    Eigen::Matrix4f current_gnss_pose_;

    Eigen::Matrix4f lidar_to_gnss_;
    bool has_gnss_data_;
    bool use_gnss_;
};

#endif
