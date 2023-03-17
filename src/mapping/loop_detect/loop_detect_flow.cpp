#include "lidar_RTK/mapping/loop_detect/loop_detect_flow.hpp"


LoopDetectFlow::LoopDetectFlow(ros::NodeHandle& nh, std::string backend_topic_name) :nh_(nh) {
    key_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, backend_topic_name, 100000);
    loop_pub_ptr_ = std::make_shared<LoopPublisher>(nh_, "/loop_pose", "map", 100);
    loop_detect_ptr_ = std::make_shared<LoopDetect>();
}


bool LoopDetectFlow::run() {
    if (!readData())
        return false;

    while (hasData()) {
        if (!validData())
            continue;

        if (detectLoop())
            publishLoop();
    }

    return true;
}


bool LoopDetectFlow::readData() {
    key_pose_sub_ptr_->ParseData(poses_data_buff_);
    return true;
}


bool LoopDetectFlow::hasData() {
    return !poses_data_buff_.empty();
}


bool LoopDetectFlow::validData() {
    key_poses_.push_back(poses_data_buff_.front().pose);
    poses_data_buff_.pop_front();
    return true;
}


bool LoopDetectFlow::detectLoop() {
    if (loop_detect_ptr_->update(key_poses_)) {
        loop_detect_ptr_->getLoopTransMatrix(old_index_, loop_transform_);
        new_index_ = key_poses_.size() - 1;
        return true;
    }
    return false;
}


bool LoopDetectFlow::publishLoop() {
    loop_pub_ptr_->publish(old_index_, new_index_, loop_transform_);
    return true;
}
