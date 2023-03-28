#include "lidar_RTK/subscriber/path_subscriber.hpp"


PathSubscriber::PathSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PathSubscriber::msg_callback, this);
}

void PathSubscriber::msg_callback(const nav_msgs::PathConstPtr& path_msg_ptr) {
    buff_mutex_.lock();
    nav_msgs::Path path_data = *path_msg_ptr;
    new_path_data_.push_back(path_data);
    buff_mutex_.unlock();
}

void PathSubscriber::ParseData(std::deque<nav_msgs::Path>& path_data_buff) {
    buff_mutex_.lock();
    if (new_path_data_.size() > 0) {
        path_data_buff.insert(path_data_buff.end(), new_path_data_.begin(), new_path_data_.end());
        new_path_data_.clear();
    }
    buff_mutex_.unlock();
}