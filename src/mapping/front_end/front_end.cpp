#include "lidar_RTK/mapping/front_end/front_end.h"


FrontEnd::FrontEnd() {
    registration_ptr_ = std::make_shared<RegistrationNDT>();
    frame_filter_ptr_ = pcl::make_shared<pcl::VoxelGrid<PointcloudData::POINT>>();
    local_map_filter_ptr_ = pcl::make_shared<pcl::VoxelGrid<PointcloudData::POINT>>();

    frame_filter_ptr_->setLeafSize(1.3f, 1.3f, 1.3f);
    local_map_filter_ptr_->setLeafSize(0.6f, 0.6f, 0.6f);

    init_pose_ = Eigen::Matrix4f::Identity();
    keyframe_dis_ = 2.0;
    local_frame_num_ = 20;
}


bool FrontEnd::update(const PointcloudData &cloud_data, Eigen::Matrix4f &cloud_pose) {
    std::vector<int> indices;
    static Eigen::Matrix4f step_pose          = init_pose_;
    static Eigen::Matrix4f predict_pose       = init_pose_;
    static Eigen::Matrix4f last_pose          = init_pose_;
    static Eigen::Matrix4f last_keyframe_pose = init_pose_;
    PointcloudData::CLOUD_PTR filtered_cloud_ptr(new PointcloudData::CLOUD());
    PointcloudData::CLOUD_PTR result_cloud_ptr(new PointcloudData::CLOUD());

    // Remove NaN point
    pcl::removeNaNFromPointCloud(*(cloud_data.cloud_ptr), *(current_frame_.cloud_data.cloud_ptr), indices);
    current_frame_.cloud_data.time = cloud_data.time;

    // First frame
    if (local_map_frames_.empty()) {
        current_frame_.pose = Eigen::Matrix4f::Identity();
        local_map_frames_.push_back(current_frame_);
        updateLocalMap(current_frame_);
        return true;
    }

    // Voxel grid filter
    frame_filter_ptr_->setInputCloud(current_frame_.cloud_data.cloud_ptr);
    frame_filter_ptr_->filter(*filtered_cloud_ptr);

    // Matching
    registration_ptr_->scanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // Predict next pose
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // Update loacal map
    if ((last_keyframe_pose(0,3) - current_frame_.pose(0,3)) * (last_keyframe_pose(0,3) - current_frame_.pose(0,3)) +
        (last_keyframe_pose(1,3) - current_frame_.pose(1,3)) * (last_keyframe_pose(1,3) - current_frame_.pose(1,3)) +
        (last_keyframe_pose(2,3) - current_frame_.pose(2,3)) * (last_keyframe_pose(2,3) - current_frame_.pose(2,3)) > keyframe_dis_) {
        updateLocalMap(current_frame_);
    }
    return true;
}


bool FrontEnd::updateLocalMap(const Frame& new_keyframe) {
    Frame keyframe = new_keyframe;
    keyframe.cloud_data.cloud_ptr.reset(new PointcloudData::CLOUD(*(new_keyframe.cloud_data.cloud_ptr)));

    // Sliding window
    PointcloudData::CLOUD_PTR transformed_cloud_ptr(new PointcloudData::CLOUD());
    local_map_frames_.push_back(keyframe);
    while (local_map_frames_.size() > local_frame_num_) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new PointcloudData::CLOUD());
    for (size_t index = 0; index < local_map_frames_.size(); ++index) {
        pcl::transformPointCloud(*(local_map_frames_.at(index).cloud_data.cloud_ptr),
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(index).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    // Voxel grid filter
    if (local_map_frames_.size() >= 10) {
        PointcloudData::CLOUD_PTR filtered_local_map_ptr(new PointcloudData::CLOUD());
        local_map_filter_ptr_->setInputCloud(local_map_ptr_);
        local_map_filter_ptr_->filter(*filtered_local_map_ptr);
        registration_ptr_->setInputTarget(filtered_local_map_ptr);
    } else {
        registration_ptr_->setInputTarget(local_map_ptr_);
    }
    
    return true;
}
