#ifndef LIDAR_RTK_MAPPING_BACK_END_BACK_END_H_
#define LIDAR_RTK_MAPPING_BACK_END_BACK_END_H_

#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/models/optimizer/optimizer_g2o.hpp"
#include "deque"

#include "ros/message.h"

class BackEnd {
public:
    BackEnd();

    bool update(PoseData cur_frontend_pose, PoseData cur_gnss_pose);
    bool isKeyFrame(PoseData cur_frontend_pose);
    bool isOptimized();
    Eigen::Isometry3d toIsometry(Eigen::Matrix4d matrix);

    Eigen::Matrix4f getLatestOptimizedPose();
    bool getOptimizedPoses(std::deque<Eigen::Matrix4f> &poses);

private:
    std::shared_ptr<OptimizerG2O> optimizer_;
    Eigen::Matrix4d last_key_pose_;
    Eigen::Matrix4d cur_key_pose_;
    Eigen::Matrix4d delta_pose_;
    float key_frame_dis_;
    int new_key_frame_cnt_;
    int new_gnss_cnt_;

    bool is_optimized_;

    std::deque<Eigen::Matrix4f> optimized_poses_;
};


#endif
