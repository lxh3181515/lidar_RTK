#include "lidar_RTK/subscriber/pointcloud_subscriber.h"


PointcloudSubscriber::PointcloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PointcloudSubscriber::msgCB, this);
}

void PointcloudSubscriber::msgCB(const sensor_msgs::PointCloud2ConstPtr & pointcloud_msg_ptr) {
    buff_mutex_.lock();
    PointcloudData tmp_pointcloud_data;
    tmp_pointcloud_data.time = pointcloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*pointcloud_msg_ptr, *(tmp_pointcloud_data.cloud_ptr));

    pointcloud_data_.push_back(tmp_pointcloud_data);
    buff_mutex_.unlock();
}

void PointcloudSubscriber::parseData(std::deque<PointcloudData> &deque_pointcloud_data) {
    buff_mutex_.lock();
    if (!pointcloud_data_.empty()) {
        deque_pointcloud_data.insert(deque_pointcloud_data.end(), pointcloud_data_.begin(), pointcloud_data_.end());
        pointcloud_data_.clear();
    }
    buff_mutex_.unlock();
}
