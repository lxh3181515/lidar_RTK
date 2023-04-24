#ifndef LIDAR_RTK_MAPPING_VIEWER_VIEWER_FLOW_HPP_
#define LIDAR_RTK_MAPPING_VIEWER_VIEWER_FLOW_HPP_

#include <ros/ros.h>
#include <deque>
#include <yaml-cpp/yaml.h>

#include "lidar_RTK/subscriber/path_subscriber.hpp"
#include "lidar_RTK/publisher/pointcloud_publisher.h"
#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/mapping/viewer/viewer.hpp"

class ViewerFlow {
public:
    ViewerFlow(ros::NodeHandle& nh);

    bool run();
    bool readData();
    bool hasData();
    bool validData();
    bool publishData();

private:
    ros::NodeHandle nh_;

    std::shared_ptr<PathSubscriber> path_sub_ptr_;
    std::shared_ptr<PointcloudPublisher> map_pub_ptr_;
    std::shared_ptr<PointcloudPublisher> scan_pub_ptr_;
    std::shared_ptr<Viewer> viewer_ptr_;
    
    std::deque<nav_msgs::Path> path_data_buff_;
    std::deque<Eigen::Matrix4f> optimized_poses_;

};

#endif
