#ifndef LIDAR_RTK_MAPPING_FRONT_END_FRONT_END_H_
#define LIDAR_RTK_MAPPING_FRONT_END_FRONT_END_H_

#include <deque>
#include <yaml-cpp/yaml.h>

#include "Eigen/Dense"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/ndt.h"

#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "lidar_RTK/models/registration/registration.h"
#include "lidar_RTK/models/registration/registration_ndt.h"

class FrontEnd {
public:
    struct Frame {
        PointcloudData cloud_data;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    };
    
public:
    FrontEnd();
    bool update(const PointcloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
    bool updateLocalMap(const Frame& new_keyframe);

private:
    pcl::VoxelGrid<PointcloudData::POINT>::Ptr frame_filter_ptr_;
    pcl::VoxelGrid<PointcloudData::POINT>::Ptr local_map_filter_ptr_;
    std::shared_ptr<Registration> registration_ptr_;
    
    Frame current_frame_;
    Eigen::Matrix4f init_pose_;
    float keyframe_dis_;    

    // Local map
    std::deque<Frame> local_map_frames_;
    PointcloudData::CLOUD_PTR local_map_ptr_;
    size_t local_frame_num_;
};

#endif
