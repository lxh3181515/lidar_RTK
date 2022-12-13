#include "lidar_RTK/subscriber/velocity_subscriber.h"


VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msgCB, this);
}

void VelocitySubscriber::msgCB(const geometry_msgs::TwistStampedConstPtr & twist_msg_ptr) {
    buff_mutex_.lock();

    VelocityData tmp_velocity_data;
    tmp_velocity_data.time = twist_msg_ptr->header.stamp.toSec();
    tmp_velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    tmp_velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    tmp_velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;
    tmp_velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    tmp_velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    tmp_velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;
    velocity_data_.push_back(tmp_velocity_data);

    buff_mutex_.unlock();
}

void VelocitySubscriber::parseData(std::deque<VelocityData> &deque_velocity_data) {
    buff_mutex_.lock();

    if (!velocity_data_.empty()) {
        deque_velocity_data.insert(deque_velocity_data.end(), velocity_data_.begin(), velocity_data_.end());
        velocity_data_.clear();
    }

    buff_mutex_.unlock();
}
