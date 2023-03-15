#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"


BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string frontend_topic_name, std::string backend_topic_name):nh_(nh) {
    frontend_sub_ = std::make_shared<OdometrySubscriber>(nh_, frontend_topic_name, 100000);
    gnss_sub_     = std::make_shared<OdometrySubscriber>(nh_, "/synced_gnss", 100000);
    backend_pub_  = std::make_shared<OdometryPublisher>(nh_, backend_topic_name, "map", "lidar", 100);
    corrected_gnss_pub_ = std::make_shared<OdometryPublisher>(nh_, "/key_gnss", "map", "lidar", 100);
    back_end_ptr_ = std::make_shared<BackEnd>();
}


bool BackEndFlow::run() {
    if (!readData())
        return false;
    
    while (hasData()) {
        if (!validData())
            continue;
        if (updateGraph())
            publishData();
    }
    return true;
}


bool BackEndFlow::readData() {
    frontend_sub_->ParseData(frontend_data_buff_);
    gnss_sub_->ParseData(gnss_data_buff_);
    return true;
}


bool BackEndFlow::hasData() {
    return !(frontend_data_buff_.empty() || gnss_data_buff_.empty());
}


bool BackEndFlow::validData() {
    cur_frontend_data_ = frontend_data_buff_.front();
    cur_gnss_data_ = gnss_data_buff_.front();
    
    // 时间同步
    double diff_time = cur_frontend_data_.time - cur_gnss_data_.time;
    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    } else if (diff_time < -0.05) {
        frontend_data_buff_.pop_front();
        return false;
    }
    
    frontend_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return true;
}


bool BackEndFlow::updateGraph() {
    static bool is_first_data = 1;
    static Eigen::Matrix4f t_gnss_lidar;

    // GNSS初始不为I，进行矫正
    if (is_first_data) {
        is_first_data = 0;
        t_gnss_lidar = cur_frontend_data_.pose * cur_gnss_data_.pose.inverse();
    }
    cur_gnss_data_.pose = t_gnss_lidar * cur_gnss_data_.pose;

    return back_end_ptr_->update(cur_frontend_data_, cur_gnss_data_);
}


bool BackEndFlow::publishData() {
    backend_pub_->publish(back_end_ptr_->getLatestOptimizedPose(), cur_frontend_data_.time);
    corrected_gnss_pub_->publish(cur_gnss_data_.pose, cur_gnss_data_.time);
    return true;
}
