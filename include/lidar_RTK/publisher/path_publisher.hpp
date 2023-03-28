#ifndef LIDAR_RTK_PUBLISHER_PATH_PUBLISHER_H_
#define LIDAR_RTK_PUBLISHER_PATH_PUBLISHER_H_

#include <ros/ros.h>
#include <deque>
#include "Eigen/Dense"
#include "nav_msgs/Path.h"


class PathPublisher {
public:
    PathPublisher(ros::NodeHandle& nh, 
                    std::string topic_name, 
                    std::string base_frame_id,
                    int buff_size);
    PathPublisher() = default;

    void publish(const std::deque<Eigen::Matrix4f>& transform_matrixs, double time);
    void publish(const std::deque<Eigen::Matrix4f>& transform_matrixs);

    bool hasSubscribers();

private:
    void publishData(const std::deque<Eigen::Matrix4f>& transform_matrixs, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
};

#endif
