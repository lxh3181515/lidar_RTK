#include "lidar_RTK/subscriber/imu_subscriber.h"

IMUSubscriber::IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msgCB, this);
}

void IMUSubscriber::msgCB(const sensor_msgs::ImuConstPtr & imu_msg_ptr) {
    buff_mutex_.lock();
    IMUData tmp_imu_data;
    tmp_imu_data.time = imu_msg_ptr->header.stamp.toSec();

    tmp_imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    tmp_imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    tmp_imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    tmp_imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    tmp_imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    tmp_imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    tmp_imu_data.orientation.x = imu_msg_ptr->orientation.x;
    tmp_imu_data.orientation.y = imu_msg_ptr->orientation.y;
    tmp_imu_data.orientation.z = imu_msg_ptr->orientation.z;
    tmp_imu_data.orientation.w = imu_msg_ptr->orientation.w;

    imu_data_.push_back(tmp_imu_data);
    buff_mutex_.unlock();
}

void IMUSubscriber::parseData(std::deque<IMUData> &deque_imu_data) {
    buff_mutex_.lock();
    if (!imu_data_.empty()) {
        deque_imu_data.insert(deque_imu_data.end(), imu_data_.begin(), imu_data_.end());
        imu_data_.clear();
    }
    buff_mutex_.unlock();
}
