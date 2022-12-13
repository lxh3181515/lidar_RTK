#include "lidar_RTK/subscriber/gnss_subscriber.h"

GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msgCB, this);
}

void GNSSSubscriber::msgCB(const sensor_msgs::NavSatFixConstPtr & nav_sat_fix_ptr) {
    buff_mutex_.lock();

    GNSSData tmp_gnss_data;
    tmp_gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    tmp_gnss_data.latitude = nav_sat_fix_ptr->latitude;
    tmp_gnss_data.longitude = nav_sat_fix_ptr->longitude;
    tmp_gnss_data.altitude = nav_sat_fix_ptr->altitude;
    tmp_gnss_data.status = nav_sat_fix_ptr->status.status;
    tmp_gnss_data.service = nav_sat_fix_ptr->status.service;
    gnss_data_.push_back(tmp_gnss_data);

    buff_mutex_.unlock();
}

void GNSSSubscriber::parseData(std::deque<GNSSData> &deque_gnss_data) {
    buff_mutex_.lock();

    if (!gnss_data_.empty()) {
        deque_gnss_data.insert(deque_gnss_data.end(), gnss_data_.begin(), gnss_data_.end());
        gnss_data_.clear();
    }

    buff_mutex_.unlock();
}
