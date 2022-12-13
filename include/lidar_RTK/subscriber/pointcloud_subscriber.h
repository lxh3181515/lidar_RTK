#ifndef LIDAR_RTK_SUBSCRIBER_POINTCLOUD_SUBSCRIBER_H_
#define LIDAR_RTK_SUBSCRIBER_POINTCLOUD_SUBSCRIBER_H_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <lidar_RTK/sensor_data/pointcloud_data.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class PointcloudSubscriber {
public:
    PointcloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    PointcloudSubscriber() = default;
    void parseData(std::deque<PointcloudData> &deque_pointcloud_data);

private:
    void msgCB(const sensor_msgs::PointCloud2ConstPtr & pointcloud_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<PointcloudData> pointcloud_data_;
    std::mutex buff_mutex_;
};

#endif
