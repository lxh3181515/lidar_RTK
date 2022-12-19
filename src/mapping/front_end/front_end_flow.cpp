#include "lidar_RTK/mapping/front_end/front_end_flow.h"


FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, std::string pointcloud_topic, std::string odom_topic) : nh_(nh) {
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, pointcloud_topic, 100000);
    odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, odom_topic, "map", "lidar", 100);
    front_end_ptr_ = std::make_shared<FrontEnd>();

    current_pose_ = Eigen::Matrix4f::Identity();
}


bool FrontEndFlow::run() {
    if (!readData())
        return false;

    while (hasData()) {
        if (!isValidData())
            continue;

        if (updateLaserOdom())
            publishOdom();
    }
    return true;
}


bool FrontEndFlow::readData() {
    pointcloud_sub_ptr_->parseData(pointcloud_data_buff_);
    return true;
}


bool FrontEndFlow::hasData() {
    return !pointcloud_data_buff_.empty();
}


bool FrontEndFlow::isValidData() {
    current_pointcloud_data_ = pointcloud_data_buff_.front();
    pointcloud_data_buff_.pop_front();
    return true;
}


bool FrontEndFlow::updateLaserOdom() {
    return front_end_ptr_->update(current_pointcloud_data_, current_pose_);
}


bool FrontEndFlow::publishOdom() {
    odom_pub_ptr_->publish(current_pose_);
    return true;
}
