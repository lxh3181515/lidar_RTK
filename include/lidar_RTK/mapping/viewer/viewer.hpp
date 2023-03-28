#ifndef LIDAR_RTK_MAPPING_VIEWER_VIEWER_HPP_
#define LIDAR_RTK_MAPPING_VIEWER_VIEWER_HPP_

#include "lidar_RTK/sensor_data/pose_data.hpp"
#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "lidar_RTK/tools/file_manager.hpp"

#include <deque>
#include <ros/message.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>

class Viewer {
public:
    Viewer();

    bool getCurMap(std::deque<Eigen::Matrix4f> optimized_poses, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr);
    bool getCurScan(int pose_index, 
                    Eigen::Matrix4f transform, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr);

private:
    pcl::VoxelGrid<pcl::PointXYZ>::Ptr filter_ptr_;
    std::string file_path_;

};

#endif
