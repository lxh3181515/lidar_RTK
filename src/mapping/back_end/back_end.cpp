#include "lidar_RTK/mapping/back_end/back_end.h"


BackEnd::BackEnd() {
    optimizer_ptr_ = std::make_shared<OptimizerG2O>();
    key_frame_dis_ = 1.0;
    new_key_frame_cnt_ = 0;
    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    is_optimized_ = false;

    file_path_ = "/media/lxhong/Datasets";

    FileManager::CreateDirectory(file_path_ + "/slam_data");
    FileManager::InitDirectory(file_path_ + "/slam_data/key_frames", "关键帧点云");
    FileManager::InitDirectory(file_path_ + "/slam_data/trajectory", "轨迹文件");
    FileManager::CreateFile(ground_truth_ofs_, file_path_ + "/slam_data/trajectory" + "/ground_truth.txt");
    FileManager::CreateFile(laser_odom_ofs_, file_path_ + "/slam_data/trajectory" + "/laser_odom.txt");
    FileManager::CreateFile(optimized_pose_ofs_, file_path_ + "/slam_data/trajectory" + "/optimized.txt");
}


bool BackEnd::update(PoseData cur_frontend_pose, 
                     PoseData cur_gnss_pose, 
                     PointcloudData cur_cloud) {
    if (!isKeyFrame(cur_frontend_pose))
        return false;

    int node_num = optimizer_ptr_->getNodeNum();
    cur_key_pose_ = cur_frontend_pose.pose.cast<double>();
    delta_pose_ = last_key_pose_.inverse() * cur_key_pose_;

    // 保存关键帧点云
    std::string file_path = file_path_ + "/slam_data/key_frames/key_frame_" + std::to_string(node_num) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *(cur_cloud.cloud_ptr));

    // 保存轨迹
    savePose(ground_truth_ofs_, cur_gnss_pose.pose);
    savePose(laser_odom_ofs_, cur_frontend_pose.pose);

    // 激光雷达里程计
    Eigen::Isometry3d isometry_cur_pose = toIsometry(cur_key_pose_);
    Eigen::Isometry3d isometry_measurement = toIsometry(delta_pose_);
    Eigen::VectorXd noise_odom(6);
    noise_odom << 0.5, 0.5, 0.5, 0.001, 0.001, 0.001;
    if (node_num == 0)
        optimizer_ptr_->addSE3Node(isometry_cur_pose, true);
    else {
        optimizer_ptr_->addSE3Node(isometry_cur_pose, false);
        optimizer_ptr_->addSE3Edge(node_num - 1, node_num, isometry_measurement, noise_odom);
    }
    new_key_frame_cnt_++;

    // GPS
    if (new_key_frame_cnt_ > 100) {
        Eigen::VectorXd noise_gnss(3);
        noise_gnss << 2.0, 2.0, 2.0;
        optimizer_ptr_->addSE3PriorXYZEdge(node_num, cur_gnss_pose.pose.block<3, 1>(0, 3).cast<double>(), noise_gnss);
        new_gnss_cnt_++;
    }

    // 达到次数，进行优化
    if (new_gnss_cnt_ > 0 || new_loop_cnt_ > 5) {
        optimizer_ptr_->optimize();
        new_key_frame_cnt_ = 0;
        new_gnss_cnt_ = 0;
        new_loop_cnt_ = 0;
        ROS_INFO("OPTIMIZE.");
        optimizer_ptr_->getOptimizedPoses(optimized_poses_);
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


bool BackEnd::update(PoseData cur_frontend_pose, 
                     PointcloudData cur_cloud) {
    if (!isKeyFrame(cur_frontend_pose))
        return false;

    int node_num = optimizer_ptr_->getNodeNum();
    cur_key_pose_ = cur_frontend_pose.pose.cast<double>();
    delta_pose_ = last_key_pose_.inverse() * cur_key_pose_;

    // 保存关键帧点云
    std::string file_path = file_path_ + "/slam_data/key_frames/key_frame_" + std::to_string(node_num) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *(cur_cloud.cloud_ptr));

    // 保存轨迹
    savePose(laser_odom_ofs_, cur_frontend_pose.pose);

    // 激光雷达里程计
    Eigen::Isometry3d isometry_cur_pose = toIsometry(cur_key_pose_);
    Eigen::Isometry3d isometry_measurement = toIsometry(delta_pose_);
    Eigen::VectorXd noise_odom(6);
    noise_odom << 0.5, 0.5, 0.5, 0.001, 0.001, 0.001;
    if (node_num == 0)
        optimizer_ptr_->addSE3Node(isometry_cur_pose, true);
    else {
        optimizer_ptr_->addSE3Node(isometry_cur_pose, false);
        optimizer_ptr_->addSE3Edge(node_num - 1, node_num, isometry_measurement, noise_odom);
    }
    new_key_frame_cnt_++;

    // 达到次数，进行优化
    if (new_loop_cnt_ > 5) {
        optimizer_ptr_->optimize();
        new_key_frame_cnt_ = 0;
        new_loop_cnt_ = 0;
        ROS_INFO("OPTIMIZE.");
        optimizer_ptr_->getOptimizedPoses(optimized_poses_);
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


bool BackEnd::insertLoop(int old_index, int new_index, Eigen::Matrix4f transform) {
    // 回环
    Eigen::Isometry3d isometry_measurement = toIsometry(transform.cast<double>());
    Eigen::VectorXd noise_loop(6);
    noise_loop << 0.3, 0.3, 0.3, 0.001, 0.001, 0.001;
    optimizer_ptr_->addSE3Edge(old_index, new_index, isometry_measurement, noise_loop);
    new_loop_cnt_++;

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


bool BackEnd::getLatestOptimizedPose(Eigen::Matrix4f &pose) {
    if (optimized_poses_.empty())
        return false;
    else {
        pose = optimized_poses_.back();
        return true;
    }
}


bool BackEnd::getOptimizedPoses(std::deque<Eigen::Matrix4f> &poses) {
    if (optimized_poses_.empty())
        return false;

    poses.clear();
    poses.insert(poses.begin(), optimized_poses_.begin(), optimized_poses_.end());
    return true;
}


bool BackEnd::isOptimized() {
    if (is_optimized_) {
        is_optimized_ = false;
        return true;
    }
    return false;
}


bool BackEnd::savePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}


bool BackEnd::forceOptimize() {
    optimizer_ptr_->optimize();
    ROS_INFO("OPTIMIZE.");
    optimizer_ptr_->getOptimizedPoses(optimized_poses_);
    is_optimized_ = true;

    int node_num = optimized_poses_.size();
    for (int i = 0; i < node_num; i++) {
        savePose(optimized_pose_ofs_, optimized_poses_.at(i));
    }
    
    return true;
}
