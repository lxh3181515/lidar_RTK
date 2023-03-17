#ifndef LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_HPP_
#define LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_HPP_

#include <deque>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <ros/message.h>

#include "lidar_RTK/models/registration/registration_ndt.h"

class LoopDetect {
public:
    LoopDetect();

    bool update(const std::deque<Eigen::Matrix4f>& optimiezed_poses);
    bool hasLoop(const std::deque<Eigen::Matrix4f>& optimiezed_poses);
    bool getCurScan(int pose_index);
    bool getHisMap(int pose_index, const std::deque<Eigen::Matrix4f>& optimiezed_poses);
    void getLoopTransMatrix(int &index, Eigen::Matrix4f &transform);

private:
    // param
    int diff_num_;
    int poses_num_;
    int loop_step_;
    int reg_map_size_;
    double loop_dis_;
    float score_limit_;

    int loop_index_;
    Eigen::Matrix4f cur_pose_;
    Eigen::Matrix4f delta_pose_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur_scan_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_his_map_ptr_;
    pcl::VoxelGrid<pcl::PointXYZ>::Ptr filter_ptr_;

    std::shared_ptr<Registration> reg_ptr_;

    std::string file_path_;
};

#endif
