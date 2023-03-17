#ifndef LIDAR_RTK_SUBSCRIBER_LOOP_SUBSCRIBER_HPP_
#define LIDAR_RTK_SUBSCRIBER_LOOP_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class LoopSubscriber {
public:
    LoopSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    LoopSubscriber() = default;
    void parseData(std::deque<geometry_msgs::PoseWithCovarianceStamped> &deque_loop_data);

private:
    void msgCB(const geometry_msgs::PoseWithCovarianceStampedPtr & loop_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<geometry_msgs::PoseWithCovarianceStamped> loop_data_;
    std::mutex buff_mutex_;
};

#endif
