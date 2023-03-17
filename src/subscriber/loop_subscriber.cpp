#include "lidar_RTK/subscriber/loop_subscriber.hpp"

LoopSubscriber::LoopSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LoopSubscriber::msgCB, this);
}

void LoopSubscriber::msgCB(const geometry_msgs::PoseWithCovarianceStampedPtr & loop_msg_ptr) {
    buff_mutex_.lock();
    geometry_msgs::PoseWithCovarianceStamped tmp_loop_data;
    tmp_loop_data = *loop_msg_ptr;
    loop_data_.push_back(tmp_loop_data);
    buff_mutex_.unlock();
}

void LoopSubscriber::parseData(std::deque<geometry_msgs::PoseWithCovarianceStamped> &deque_loop_data) {
    buff_mutex_.lock();
    if (!loop_data_.empty()) {
        deque_loop_data.insert(deque_loop_data.end(), loop_data_.begin(), loop_data_.end());
        loop_data_.clear();
    }
    buff_mutex_.unlock();
}

