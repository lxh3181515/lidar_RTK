#include "lidar_RTK/mapping/back_end/back_end.h"
#include "lidar_RTK/global_defination/global_defination.h"


BackEnd::BackEnd() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/back_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------后端初始化-------------------" << std::endl;

    key_frame_dis_ = config_node["key_frame_distance"].as<float>();
    file_path_ = config_node["data_path"].as<std::string>();

    optimize_step_with_gnss_ = config_node["optimize_step_with_gnss"].as<int>();
    optimize_step_with_loop_ = config_node["optimize_step_with_loop"].as<int>();

    noise_odom_.resize(6);
    noise_loop_.resize(6);
    noise_gnss_.resize(3);
    for (int i = 0; i < 6; i++) {
        noise_odom_(i) = config_node["g2o_param"]["odom_edge_noise"][i].as<double>();
        noise_loop_(i) = config_node["g2o_param"]["close_loop_noise"][i].as<double>();
        if (i < 3)
            noise_gnss_(i) = config_node["g2o_param"]["gnss_noise"][i].as<double>();
    }

    optimizer_ptr_ = std::make_shared<OptimizerG2O>();
    new_key_frame_cnt_ = 0;
    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    is_optimized_ = false;

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

    // 图优化
    insertOdom(node_num, cur_key_pose_, delta_pose_);
    insertGNSS(node_num, cur_gnss_pose.pose.cast<double>());
    optimize();

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

    // 图优化
    insertOdom(node_num, cur_key_pose_, delta_pose_);
    optimize();

    last_key_pose_ = cur_key_pose_;

    return true;
}


// 激光雷达里程计
bool BackEnd::insertOdom(int index, Eigen::Matrix4d node_pose, Eigen::Matrix4d meas_pose) {
    Eigen::Isometry3d isometry_cur_pose = toIsometry(node_pose);
    Eigen::Isometry3d isometry_measurement = toIsometry(meas_pose);
    if (index == 0)
        optimizer_ptr_->addSE3Node(isometry_cur_pose, true);
    else {
        optimizer_ptr_->addSE3Node(isometry_cur_pose, false);
        optimizer_ptr_->addSE3Edge(index - 1, index, isometry_measurement, noise_odom_);
    }
    new_key_frame_cnt_++;
    return true;
}


// GNSS
bool BackEnd::insertGNSS(int index, Eigen::Matrix4d meas_pose) {
    optimizer_ptr_->addSE3PriorXYZEdge(index, meas_pose.block<3, 1>(0, 3), noise_gnss_);
    new_gnss_cnt_++;
    return true;
}


// 回环
bool BackEnd::insertLoop(int old_index, int new_index, Eigen::Matrix4f transform) {
    Eigen::Isometry3d isometry_measurement = toIsometry(transform.cast<double>());
    optimizer_ptr_->addSE3Edge(old_index, new_index, isometry_measurement, noise_loop_);
    new_loop_cnt_++;
    return true;
}


bool BackEnd::optimize() {
    if (new_gnss_cnt_ > optimize_step_with_gnss_ || new_loop_cnt_ > optimize_step_with_loop_) {
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
