#ifndef LIDAR_RTK_PUBLISHER_TRAJECTORY_PUBLISHER_HPP_
#define LIDAR_RTK_PUBLISHER_TRAJECTORY_PUBLISHER_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>
#include <nav_msgs/Path.h>

class TrajectoryPublisher {
public:
    TrajectoryPublisher(ros::NodeHandle& nh, 
                       std::string topic_name, 
                       std::string base_frame_id,
                       int buff_size);

    void publish(const std::deque<Eigen::Matrix4f> & transform_matrix, double time);
    void publish(const std::deque<Eigen::Matrix4f> & transform_matrix);

private:
    void publishData(const std::deque<Eigen::Matrix4f> & transform_matrix, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
    std::string base_frame_id_;
};

#endif
