#include "lidar_RTK/mapping/front_end/front_end_flow.h"
#include "lidar_RTK/global_defination/global_defination.h"


FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh) : nh_(nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/topic.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, 
                                                                 config_node["pointcloud_topic"].as<std::string>(), 
                                                                 100000);
    odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, 
                                                        config_node["lidar_odom_topic"]["name"].as<std::string>(), 
                                                        config_node["lidar_odom_topic"]["base_frame"].as<std::string>(), 
                                                        config_node["lidar_odom_topic"]["child_frame"].as<std::string>(), 
                                                        100);
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
    odom_pub_ptr_->publish(current_pose_, current_pointcloud_data_.time);
    return true;
}
