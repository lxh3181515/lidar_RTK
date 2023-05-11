#ifndef LIDAR_RTK_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define LIDAR_RTK_SUBSCRIBER_GNSS_SUBSCRIBER_H_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "lidar_RTK/sensor_data/gnss_data.h"

class GNSSSubscriber {
public:
    GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void parseData(std::deque<GNSSData> &deque_gnss_data);

private:
    void msgCB(const sensor_msgs::NavSatFixConstPtr & nav_sat_fix_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<GNSSData> gnss_data_;
    std::mutex buff_mutex_;

    int first_3_data_;
};

#endif
