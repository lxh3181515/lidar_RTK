#include "lidar_RTK/publisher/pointcloud_publisher.h"


PointcloudPublisher::PointcloudPublisher(ros::NodeHandle& nh,
                                        std::string topic_name,
                                        std::string frame_id,
                                        size_t buff_size) 
    : nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}


void PointcloudPublisher::publish(PointcloudData::CLOUD_PTR& cloud_ptr_input, double time) {
    publishData(cloud_ptr_input, ros::Time().fromSec(time));
}


void PointcloudPublisher::publish(PointcloudData::CLOUD_PTR& cloud_ptr_input) {
    publishData(cloud_ptr_input, ros::Time::now());
}


void PointcloudPublisher::publishData(PointcloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());

    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}


bool PointcloudPublisher::hasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
