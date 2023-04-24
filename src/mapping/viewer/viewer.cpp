#include "lidar_RTK/mapping/viewer/viewer.hpp"
#include "lidar_RTK/global_defination/global_defination.h"


Viewer::Viewer() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/viewer.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    filter_ptr_ = pcl::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
    filter_ptr_->setLeafSize(config_node["global_map"]["leaf_size"][0].as<float>(),
                             config_node["global_map"]["leaf_size"][1].as<float>(),
                             config_node["global_map"]["leaf_size"][2].as<float>());
    file_path_ = config_node["data_path"].as<std::string>() + "/slam_data/key_frames";
}


bool Viewer::getCurMap(std::deque<Eigen::Matrix4f> optimized_poses, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>());

    int size = optimized_poses.size();
    map_cloud_ptr->clear();
    for (int i = 0; i < size; i++) {
        tmp_cloud_ptr_->clear();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_ + "/key_frame_" + std::to_string(i) + ".pcd", 
                                                *tmp_cloud_ptr_) == -1) {
            ROS_ERROR("Can not load PCD file!");
            return false;
        }

        // down sample
        filter_ptr_->setInputCloud(tmp_cloud_ptr_);
        filter_ptr_->filter(*tmp_cloud_ptr_);

        // transform
        pcl::transformPointCloud(*tmp_cloud_ptr_, *tmp_cloud_ptr_, optimized_poses.at(i));

        *map_cloud_ptr += *tmp_cloud_ptr_;
    }

    // down sample
    filter_ptr_->setInputCloud(map_cloud_ptr);
    filter_ptr_->filter(*map_cloud_ptr);

    return true;
}


bool Viewer::getCurScan(int pose_index, 
                        Eigen::Matrix4f transform, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr) {
    scan_cloud_ptr->clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_ + "/key_frame_" + std::to_string(pose_index) + ".pcd", 
                                            *scan_cloud_ptr) == -1) {
        ROS_ERROR("Can not load PCD file!");
        return false;
    }
    // down sample
    filter_ptr_->setInputCloud(scan_cloud_ptr);
    filter_ptr_->filter(*scan_cloud_ptr);

    pcl::transformPointCloud(*scan_cloud_ptr, *scan_cloud_ptr, transform);

    return true;
}
