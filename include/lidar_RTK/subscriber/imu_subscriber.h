#ifndef LIDAR_RTK_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define LIDAR_RTK_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lidar_RTK/sensor_data/imu_data.h"

class IMUSubscriber {
public:
    IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void parseData(std::deque<IMUData> &deque_imu_data);

private:
    void msgCB(const sensor_msgs::ImuConstPtr & imu_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUData> imu_data_;
    std::mutex buff_mutex_;
};

#endif
