#ifndef LIDAR_RTK_SUBSCRIBER_PATH_SUBSCRIBER_HPP_
#define LIDAR_RTK_SUBSCRIBER_PATH_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Path.h>


class PathSubscriber {
  public:
    PathSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    PathSubscriber() = default;
    void ParseData(std::deque<nav_msgs::Path>& deque_path_data);

  private:
    void msg_callback(const nav_msgs::PathConstPtr& path_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::Path> new_path_data_;

    std::mutex buff_mutex_; 
};
#endif