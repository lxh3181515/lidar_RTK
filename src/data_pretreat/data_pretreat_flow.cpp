#include "lidar_RTK/data_pretreat/data_pretreat_flow.h"


DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_        = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    gnss_sub_ptr_       = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    velocity_sub_ptr_   = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);

    pointcloud_pub_ptr_ = std::make_shared<PointcloudPublisher>(nh, "/synced_cloud", "velo_link", 100);
    odometry_pub_ptr_   = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "map", "velo_link", 100);

    lidar_to_imu_ptr_   = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
    lidar_to_imu_       = Eigen::Matrix4f::Identity();

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}


bool DataPretreatFlow::run() {
    if (!readData())
        return false;

    if (!initGNSS())
        return false;

    if (!initCalibration())
        return false;

    // Make sure all data in the buff output.
    while (hasData()) {
        if (!isValidData())
            continue;

        transformData();
        publishData();

        // printDataBuff();
    }
    return true;
}


bool DataPretreatFlow::readData() {
    pointcloud_sub_ptr_->parseData(pointcloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // Parse data
    imu_sub_ptr_->parseData(unsynced_imu_);
    velocity_sub_ptr_->parseData(unsynced_velocity_);
    gnss_sub_ptr_->parseData(unsynced_gnss_);

    if (pointcloud_data_buff_.empty())
        return false;

    // Sync time
    double pointcloud_time = pointcloud_data_buff_.front().time;
    bool is_valid_imu      = IMUData::syncData(unsynced_imu_, imu_data_buff_, pointcloud_time);
    bool is_valid_gnss     = GNSSData::syncData(unsynced_gnss_, gnss_data_buff_, pointcloud_time);
    bool is_valid_velocity = VelocityData::syncData(unsynced_velocity_, velocity_data_buff_, pointcloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!is_valid_imu || !is_valid_gnss || !is_valid_velocity) {
            pointcloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}


bool DataPretreatFlow::initGNSS() {
    static bool gnss_inited = false;

    if (!gnss_inited && !gnss_data_buff_.empty()) {
        gnss_data_buff_.front().initOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}


bool DataPretreatFlow::initCalibration() {
    static bool calibration_inited = false;

    if (!calibration_inited && lidar_to_imu_ptr_->LookupData(lidar_to_imu_))
        calibration_inited = true;

    return calibration_inited;
}


bool DataPretreatFlow::hasData() {
    if (   pointcloud_data_buff_.empty()  
        || imu_data_buff_.empty()  
        || gnss_data_buff_.empty()  
        || velocity_data_buff_.empty())
        return false;
    
    return true;
}


bool DataPretreatFlow::isValidData() {
    current_pointcloud_data_ = pointcloud_data_buff_.front();
    current_imu_data_        = imu_data_buff_.front();
    current_gnss_data_       = gnss_data_buff_.front();
    current_velocity_data_   = velocity_data_buff_.front();

    double diff_imu_time      = current_pointcloud_data_.time - current_imu_data_.time;
    double diff_gnss_time     = current_pointcloud_data_.time - current_gnss_data_.time;
    double diff_velocity_time = current_pointcloud_data_.time - current_velocity_data_.time;

    if (diff_imu_time < -0.05 || diff_gnss_time < -0.05 || diff_velocity_time < -0.05) {
        pointcloud_data_buff_.pop_front();
        return false;
    }
    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }
    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }
    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    pointcloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    velocity_data_buff_.pop_front();

    return true;
}


bool DataPretreatFlow::transformData() {
    current_gnss_pose_ = Eigen::Matrix4f::Identity();

    // Get gnss pose
    current_gnss_data_.updateXYZ();
    current_gnss_pose_(0, 3) = current_gnss_data_.local_E;
    current_gnss_pose_(1, 3) = current_gnss_data_.local_N;
    current_gnss_pose_(2, 3) = current_gnss_data_.local_U;
    current_gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.orientation2Matrix();
    current_gnss_pose_ *= lidar_to_imu_;

    // Distortion adjust
    current_velocity_data_.transformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->setMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->adjustCloud(current_pointcloud_data_.cloud_ptr, current_pointcloud_data_.cloud_ptr);

    return true;
}


bool DataPretreatFlow::publishData() {
    pointcloud_pub_ptr_->publish(current_pointcloud_data_.cloud_ptr, current_pointcloud_data_.time);
    odometry_pub_ptr_->publish(current_gnss_pose_, current_gnss_data_.time);

    return true;
}


void DataPretreatFlow::printDataBuff() {
    ROS_INFO("PT, IMU, GNSS, V: %d, %d, %d, %d",
            (int)pointcloud_data_buff_.size(), 
            (int)imu_data_buff_.size(), 
            (int)gnss_data_buff_.size(), 
            (int)velocity_data_buff_.size());
    ROS_INFO("TIME:%.2f", current_pointcloud_data_.time);
}
