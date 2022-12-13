#ifndef LIDAR_RTK_PUBLISHER_ODOMETRY_PUBLISHER_H_
#define LIDAR_RTK_PUBLISHER_ODOMETRY_PUBLISHER_H_

#include <ros/ros.h>
#include "Eigen/Dense"
#include "nav_msgs/Odometry.h"


class OdometryPublisher {
public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void publish(const Eigen::Matrix4f& transform_matrix, double time);
    void publish(const Eigen::Matrix4f& transform_matrix);

    bool hasSubscribers();

private:
    void publishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};

#endif
