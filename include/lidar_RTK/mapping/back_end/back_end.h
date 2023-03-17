#ifndef LIDAR_RTK_MAPPING_BACK_END_BACK_END_H_
#define LIDAR_RTK_MAPPING_BACK_END_BACK_END_H_

#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "lidar_RTK/models/optimizer/optimizer_g2o.hpp"
#include "lidar_RTK/tools/file_manager.hpp"

#include <deque>
#include <ros/message.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class BackEnd {
public:
    BackEnd();

    bool update(PoseData cur_frontend_pose, 
                PoseData cur_gnss_pose, 
                PointcloudData cur_cloud);
    bool insertLoop(int old_index, int new_index, Eigen::Matrix4f transform);
    bool isKeyFrame(PoseData cur_frontend_pose);
    bool isOptimized();
    Eigen::Isometry3d toIsometry(Eigen::Matrix4d matrix);

    bool getLatestOptimizedPose(Eigen::Matrix4f &pose);
    bool getOptimizedPoses(std::deque<Eigen::Matrix4f> &poses);

    bool savePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);
    bool forceOptimize();

private:
    std::shared_ptr<OptimizerG2O> optimizer_ptr_;
    Eigen::Matrix4d last_key_pose_;
    Eigen::Matrix4d cur_key_pose_;
    Eigen::Matrix4d delta_pose_;
    float key_frame_dis_;
    int new_key_frame_cnt_;
    int new_gnss_cnt_;
    int new_loop_cnt_;

    bool is_optimized_;

    std::deque<Eigen::Matrix4f> optimized_poses_;

    std::string file_path_;
    std::ofstream ground_truth_ofs_;
    std::ofstream laser_odom_ofs_;
    std::ofstream optimized_pose_ofs_;
};


#endif
