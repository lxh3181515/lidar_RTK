#ifndef LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_FLOW_HPP_
#define LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_FLOW_HPP_

#include "lidar_RTK/mapping/loop_detect/loop_detect.hpp"
#include "lidar_RTK/subscriber/odometry_subscriber.hpp"
#include "lidar_RTK/publisher/loop_publisher.hpp"

class LoopDetectFlow {
public:
    LoopDetectFlow(ros::NodeHandle& nh, std::string backend_topic_name);

    bool run();
    bool readData();
    bool hasData();
    bool validData();
    bool detectLoop();
    bool publishLoop();

private:
    ros::NodeHandle nh_;
    std::deque<Eigen::Matrix4f> key_poses_;
    std::deque<PoseData> poses_data_buff_;

    std::shared_ptr<OdometrySubscriber> key_pose_sub_ptr_;
    std::shared_ptr<LoopPublisher> loop_pub_ptr_;
    std::shared_ptr<LoopDetect> loop_detect_ptr_;

    Eigen::Matrix4f loop_transform_;
    int old_index_;
    int new_index_;
};

#endif