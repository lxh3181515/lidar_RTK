#include "lidar_RTK/data_pretreat/gnss_pretreat_flow.hpp"


GNSSPretreatFlow::GNSSPretreatFlow(ros::NodeHandle& nh) {
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh, "/lslidar_point_cloud", 100000);
    gnss_sub_ptr_     = std::make_shared<GNSSSubscriber>(nh, "/fix", 1000000);

    pointcloud_pub_ptr_ = std::make_shared<PointcloudPublisher>(nh, "/synced_cloud", "velo_link", 100);
    odometry_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "map", "velo_link", 100);

    lidar_to_gnss_ = Eigen::Matrix4f::Identity();
    // lidar_to_gnss_(0, 3) = -0.08;
    // lidar_to_gnss_(1, 3) = -0.15;
    // lidar_to_gnss_(2, 3) = -0.35;

    has_gnss_data_ = false;
}


bool GNSSPretreatFlow::run() {
    if (!readData())
        return false;

    if (!initGNSS())
        return false;

    // Make sure all data in the buff output.
    while (hasData()) {
        if (!isValidData())
            continue;

        transformData();
        publishData();
    }
    return true;
}


bool GNSSPretreatFlow::readData() {
    pointcloud_sub_ptr_->parseData(pointcloud_data_buff_);

    static std::deque<GNSSData> unsynced_gnss_;

    // Parse data
    gnss_sub_ptr_->parseData(unsynced_gnss_);

    if (pointcloud_data_buff_.size() < 2)
        return false;

    // Sync time
    double pointcloud_time = pointcloud_data_buff_.front().time;
    bool is_valid_gnss = GNSSData::syncData(unsynced_gnss_, gnss_data_buff_, pointcloud_time);

    // ROS_INFO("is_valid_gnss:%d, gnss_data_buff_:%d, unsynced_gnss_:%d", is_valid_gnss, gnss_data_buff_.size(), unsynced_gnss_.size());

    // Init GNSS
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!is_valid_gnss) {
            if (unsynced_gnss_.front().time > pointcloud_time)
                pointcloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}


bool GNSSPretreatFlow::initGNSS() {
    static bool gnss_inited = false;

    if (!gnss_inited && !gnss_data_buff_.empty()) {
        gnss_data_buff_.front().initOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}


bool GNSSPretreatFlow::hasData() {
    if (!gnss_data_buff_.empty())
        has_gnss_data_ = 1;

    if (pointcloud_data_buff_.empty())
        return false;
    
    return true;
}


bool GNSSPretreatFlow::isValidData() {
    current_cloud_data_ = pointcloud_data_buff_.front();
    pointcloud_data_buff_.pop_front();

    if (has_gnss_data_) {
        current_gnss_data_ = gnss_data_buff_.front();
        gnss_data_buff_.pop_front();
    }

    return true;
}


bool GNSSPretreatFlow::transformData() {
    if (!has_gnss_data_) {
        return false;
    }

    static Eigen::Matrix4f t_gnss_lidar;
    static bool is_first_data = 1;

    current_gnss_pose_ <<   0.1585,   -0.9874,         0,    0,
                            0.9874,    0.1585,         0,    0,
                                0,         0,    1.0000,         0,
                                0,         0,         0,    1.0000;

    // Get gnss pose
    current_gnss_data_.updateXYZ();
    current_gnss_pose_(0, 3) = current_gnss_data_.local_E;
    current_gnss_pose_(1, 3) = current_gnss_data_.local_N;
    current_gnss_pose_(2, 3) = current_gnss_data_.local_U;
    current_gnss_pose_ *= lidar_to_gnss_;
    if (is_first_data) {
        t_gnss_lidar = current_gnss_pose_.inverse();
        is_first_data = 0;
    }
    current_gnss_pose_ = t_gnss_lidar * current_gnss_pose_;

    return true;
}


bool GNSSPretreatFlow::publishData() {
    if (current_cloud_data_.cloud_ptr->size() > 25000) {
        pointcloud_pub_ptr_->publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    }

    if (has_gnss_data_) {
        odometry_pub_ptr_->publish(current_gnss_pose_, current_gnss_data_.time);
        has_gnss_data_ = 0;
    }
    
    return true;
}
