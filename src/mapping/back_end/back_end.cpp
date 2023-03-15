#include "lidar_RTK/mapping/back_end/back_end.h"


BackEnd::BackEnd() {
    optimizer_ = std::make_shared<OptimizerG2O>();
    key_frame_dis_ = 2.0;
    new_key_frame_cnt_ = 0;
    new_gnss_cnt_ = 0;

    is_optimized_ = false;
}


bool BackEnd::update(PoseData cur_frontend_pose, PoseData cur_gnss_pose) {
    if (!isKeyFrame(cur_frontend_pose))
        return false;

    cur_key_pose_ = cur_frontend_pose.pose.cast<double>();
    delta_pose_ = last_key_pose_.inverse() * cur_key_pose_;

    // 激光雷达里程计
    int node_num = optimizer_->getNodeNum();
    Eigen::Isometry3d isometry_cur_pose = toIsometry(cur_key_pose_);
    Eigen::Isometry3d isometry_measurement = toIsometry(delta_pose_);
    Eigen::VectorXd noise_odom(6);
    noise_odom << 0.5, 0.5, 0.5, 0.001, 0.001, 0.001;
    if (node_num == 0)
        optimizer_->addSE3Node(isometry_cur_pose, true);
    else {
        optimizer_->addSE3Node(isometry_cur_pose, false);
        optimizer_->addSE3Edge(node_num - 1, node_num, isometry_measurement, noise_odom);
    }
    new_key_frame_cnt_++;

    // GPS
    Eigen::VectorXd noise_gnss(3);
    noise_gnss << 2.0, 2.0, 2.0;
    optimizer_->addSE3PriorXYZEdge(node_num, cur_gnss_pose.pose.block<3, 1>(0, 3).cast<double>(), noise_gnss);
    new_gnss_cnt_++;

    // 达到次数，进行优化
    if (new_key_frame_cnt_ > 10 || new_gnss_cnt_ > 10) {
        optimizer_->optimize();
        new_key_frame_cnt_ = 0;
        new_gnss_cnt_ = 0;
        ROS_INFO("OPTIMIZE.");
        optimizer_->getOptimizedPoses(optimized_poses_);
        is_optimized_ = true;
    } else {
        if (optimized_poses_.empty())
            optimized_poses_.push_back(cur_key_pose_.cast<float>());
        else
            optimized_poses_.push_back(optimized_poses_.back() * delta_pose_.cast<float>());
    }

    last_key_pose_ = cur_key_pose_;

    return true;
}


bool BackEnd::isKeyFrame(PoseData cur_frontend_pose) {
    static bool first_key_frame = true;
    bool is_key_frame = false;

    if (first_key_frame) {
        is_key_frame = true;
        first_key_frame = false;
        last_key_pose_ = cur_frontend_pose.pose.cast<double>();
    }

    if (fabs(cur_frontend_pose.pose(0, 3) - last_key_pose_(0, 3)) + 
        fabs(cur_frontend_pose.pose(1, 3) - last_key_pose_(1, 3)) + 
        fabs(cur_frontend_pose.pose(2, 3) - last_key_pose_(2, 3)) > key_frame_dis_) {
        is_key_frame = true;
    }

    return is_key_frame;
}


Eigen::Isometry3d BackEnd::toIsometry(Eigen::Matrix4d matrix) {
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    // output.rotate(matrix.block<3, 3>(0, 0));
    // output.translate(matrix.block<3, 1>(0, 3));
    output.matrix() = matrix;
    return output;
}


Eigen::Matrix4f BackEnd::getLatestOptimizedPose() {
    return optimized_poses_.back();
}


bool BackEnd::getOptimizedPoses(std::deque<Eigen::Matrix4f> &poses) {
    poses.clear();
    poses.insert(poses.begin(), optimized_poses_.begin(), optimized_poses_.end());
    return !poses.empty();
}


bool BackEnd::isOptimized() {
    if (is_optimized_) {
        is_optimized_ = false;
        return true;
    }
    return false;
}
