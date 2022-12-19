#ifndef LIDAR_RTK_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#define LIDAR_RTK_MAPPING_FRONT_END_FRONT_END_FLOW_H_

#include <ros/ros.h>
#include <deque>

#include "lidar_RTK/subscriber/pointcloud_subscriber.h"
#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "lidar_RTK/mapping/front_end/front_end.h"
#include "lidar_RTK/publisher/odometry_publisher.h"
#include "Eigen/Dense"

class FrontEndFlow {
public:
    FrontEndFlow(ros::NodeHandle &nh, std::string pointcloud_topic, std::string odom_topic);

    bool run();
    bool readData();
    bool hasData();
    bool isValidData();
    bool updateLaserOdom();
    bool publishOdom();

private:
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;
    std::shared_ptr<OdometryPublisher> odom_pub_ptr_;

    ros::NodeHandle nh_;

    std::deque<PointcloudData> pointcloud_data_buff_;

    PointcloudData current_pointcloud_data_;
    Eigen::Matrix4f current_pose_;
};

#endif
