#ifndef LIDAR_RTK_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
#define LIDAR_RTK_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include "lidar_RTK/sensor_data/velocity_data.h"
#include "geometry_msgs/TwistStamped.h"


class VelocitySubscriber {
public:
    VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void parseData(std::deque<VelocityData> &deque_velocity_data);

private:
    void msgCB(const geometry_msgs::TwistStampedConstPtr & twist_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> velocity_data_;
    std::mutex buff_mutex_;
};

#endif
