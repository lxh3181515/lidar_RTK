#ifndef LIDAR_RTK_TF_LISTENER_H_
#define LIDAR_RTK_TF_LISTENER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

class TFListener {
public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;
    bool LookupData(Eigen::Matrix4f& transform_matrix);

private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};

#endif
