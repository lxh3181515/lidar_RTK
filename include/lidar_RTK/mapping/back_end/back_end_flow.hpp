#ifndef LIDAR_RTK_MAPPING_BACK_END_BACK_END_HPP_
#define LIDAR_RTK_MAPPING_BACK_END_BACK_END_HPP_

#include <ros/ros.h>
#include <deque>
#include <yaml-cpp/yaml.h>

#include "lidar_RTK/subscriber/odometry_subscriber.hpp"
#include "lidar_RTK/subscriber/pointcloud_subscriber.h"
#include "lidar_RTK/subscriber/loop_subscriber.hpp"
#include "lidar_RTK/publisher/odometry_publisher.h"
#include "lidar_RTK/publisher/path_publisher.hpp"
#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/mapping/back_end/back_end.h"
#include "lidar_RTK/mapping/loop_detect/loop_detect.hpp"


class BackEndFlow {
public:
    BackEndFlow(ros::NodeHandle& nh);

    bool run();
    bool readData();
    bool hasData();
    bool validData();
    bool updateGraph();
    bool publishData();
    bool mayHaveLoop();

    bool forceOptimize();

private:
    std::shared_ptr<BackEnd> back_end_ptr_;
    std::shared_ptr<LoopDetect> loop_detect_ptr_;

    std::shared_ptr<OdometrySubscriber> frontend_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;
    std::shared_ptr<LoopSubscriber> loop_sub_ptr_;
    std::shared_ptr<OdometryPublisher> backend_pub_ptr_;
    std::shared_ptr<PathPublisher> path_pub_ptr_;

    std::deque<PoseData> frontend_data_buff_;
    std::deque<PoseData> gnss_data_buff_;
    std::deque<PointcloudData> cloud_data_buff_;
    std::deque<geometry_msgs::PoseWithCovarianceStamped> loop_data_buff_;
    ros::NodeHandle nh_;

    PoseData cur_frontend_data_;
    PoseData cur_gnss_data_;
    PointcloudData cur_cloud_data_;
    
    bool use_gnss_;
    bool use_loop_close_;
    bool have_gnss_;
    bool valid_gnss_;
};

#endif
