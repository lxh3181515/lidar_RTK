#include "lidar_RTK/mapping/loop_detect/loop_detect.hpp"


LoopDetect::LoopDetect() {
    file_path_ = "/media/lxhong/Datasets/slam_data/key_frames";

    cloud_cur_scan_ptr_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_his_map_ptr_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filter_ptr_ = pcl::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
    filter_ptr_->setLeafSize(0.3f, 0.3f, 0.3f);

    reg_ptr_ = std::make_shared<RegistrationNDT>();

    diff_num_ = 100;
    loop_dis_ = 10.0;
    loop_step_ = 5;
    reg_map_size_ = 5;
    score_limit_ = 0.3;
}


bool LoopDetect::update(const std::deque<Eigen::Matrix4f>& optimiezed_poses) {
    static int key_frame_cnt = 0;

    if (key_frame_cnt++ < loop_step_)
        return false;
    else
        key_frame_cnt = 0;

    poses_num_ = optimiezed_poses.size();
    cur_pose_ = optimiezed_poses.back();

    // 寻找回环
    if (!hasLoop(optimiezed_poses)) 
        return false;

    // 构建子图
    if (!getCurScan(poses_num_ - 1)) 
        return false;
    if (!getHisMap(loop_index_, optimiezed_poses))
        return false;    
    
    // 匹配
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reg_ptr_->setInputTarget(cloud_his_map_ptr_);
    reg_ptr_->scanMatch(cloud_cur_scan_ptr_, Eigen::Matrix4f::Identity(), tmp_cloud, delta_pose_);
    if (reg_ptr_->getScore() > score_limit_) {
        // ROS_INFO("Score:%.2f", reg_ptr_->getScore());
        return false;
    }

    return true;
}


bool LoopDetect::hasLoop(const std::deque<Eigen::Matrix4f>& optimiezed_poses) {
    if (poses_num_ < diff_num_)
        return false;

    // 计算最小距离
    int min_index = 0;
    double min_dis = loop_dis_ + 1.0;
    for (int i = 0; i < poses_num_ - diff_num_; i++) {
        double dis = fabs(optimiezed_poses.at(i)(0, 3) - cur_pose_(0, 3)) + 
                     fabs(optimiezed_poses.at(i)(1, 3) - cur_pose_(1, 3)) + 
                     fabs(optimiezed_poses.at(i)(2, 3) - cur_pose_(2, 3)) * 0.1;
        if (i == 0 || dis < min_dis) {
            min_dis = dis;
            min_index = i;
        }
    }

    if (min_dis > loop_dis_)
        return false;

    loop_index_ = min_index;
    
    return true;
}


bool LoopDetect::getCurScan(int pose_index) {
    cloud_cur_scan_ptr_->clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_ + "/key_frame_" + std::to_string(pose_index) + ".pcd", 
                                            *cloud_cur_scan_ptr_) == -1) {
        ROS_ERROR("Can not load PCD file!");
        return false;
    }

    // transform
    pcl::transformPointCloud(*cloud_cur_scan_ptr_, *cloud_cur_scan_ptr_, cur_pose_);

    // down sample
    filter_ptr_->setInputCloud(cloud_cur_scan_ptr_);
    filter_ptr_->filter(*cloud_cur_scan_ptr_);

    return true;
}


bool LoopDetect::getHisMap(int pose_index, const std::deque<Eigen::Matrix4f>& optimiezed_poses) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>());

    cloud_his_map_ptr_->clear();
    for (int i = -reg_map_size_; i <= reg_map_size_; i++) {
        if (pose_index + i < 0)
            continue;
        tmp_cloud_ptr_->clear();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_ + "/key_frame_" + std::to_string(pose_index + i) + ".pcd", 
                                                *tmp_cloud_ptr_) == -1) {
            ROS_ERROR("Can not load PCD file!");
            return false;
        }

        // transform
        pcl::transformPointCloud(*tmp_cloud_ptr_, *tmp_cloud_ptr_, optimiezed_poses.at(pose_index + i));

        *cloud_his_map_ptr_ += *tmp_cloud_ptr_;
    }

    // down sample
    filter_ptr_->setInputCloud(cloud_his_map_ptr_);
    filter_ptr_->filter(*cloud_his_map_ptr_);

    return true;
}


void LoopDetect::getLoopTransMatrix(int &index, Eigen::Matrix4f &transform) {
    index = loop_index_;
    transform = delta_pose_;
}
