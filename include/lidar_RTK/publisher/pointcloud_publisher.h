#ifndef LIDAR_RTK_PUBLISHER_POINTCLOUD_PUBLISHER_H_
#define LIDAR_RTK_PUBLISHER_POINTCLOUD_PUBLISHER_H_

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "lidar_RTK/sensor_data/pointcloud_data.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class PointcloudPublisher {
public:
    PointcloudPublisher(ros::NodeHandle& nh,
                        std::string topic_name,
                        std::string frame_id,
                        size_t buff_size);

    void publish(PointcloudData::CLOUD_PTR& cloud_ptr_input, double time);
    void publish(PointcloudData::CLOUD_PTR& cloud_ptr_input);
    bool hasSubscribers();

private:
    void publishData(PointcloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

#endif
