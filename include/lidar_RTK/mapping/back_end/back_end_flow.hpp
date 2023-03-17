#ifndef LIDAR_RTK_MAPPING_BACK_END_BACK_END_HPP_
#define LIDAR_RTK_MAPPING_BACK_END_BACK_END_HPP_

#include <ros/ros.h>
#include <deque>

#include "lidar_RTK/subscriber/odometry_subscriber.hpp"
#include "lidar_RTK/subscriber/pointcloud_subscriber.h"
#include "lidar_RTK/publisher/odometry_publisher.h"
#include "lidar_RTK/publisher/trajectory_publisher.hpp"
#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/mapping/back_end/back_end.h"
#include "lidar_RTK/mapping/loop_detect/loop_detect.hpp"


class BackEndFlow {
public:
    BackEndFlow(ros::NodeHandle& nh, std::string frontend_topic_name, std::string backend_topic_name);

    bool run();
    bool readData();
    bool hasData();
    bool validData();
    bool updateGraph();
    bool publishData();
    bool loopDetect();

    bool forceOptimize();

private:
    std::shared_ptr<BackEnd> back_end_ptr_;
    std::shared_ptr<LoopDetect> loop_detect_ptr_;

    std::shared_ptr<OdometrySubscriber> frontend_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;
    std::shared_ptr<OdometryPublisher> backend_pub_ptr_;
    std::shared_ptr<TrajectoryPublisher> traj_pub_ptr_;

    std::deque<PoseData> frontend_data_buff_;
    std::deque<PoseData> gnss_data_buff_;
    std::deque<PointcloudData> cloud_data_buff_;
    std::deque<Eigen::Matrix4f> optimized_poses_;
    ros::NodeHandle nh_;

    PoseData cur_frontend_data_;
    PoseData cur_gnss_data_;
    PointcloudData cur_cloud_data_;
    Eigen::Matrix4f cur_optimized_pose_;
};

#endif
