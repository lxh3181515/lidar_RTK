#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"


BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string frontend_topic_name, std::string backend_topic_name):nh_(nh) {
    frontend_sub_ptr_   = std::make_shared<OdometrySubscriber>(nh_, frontend_topic_name, 100000);
    gnss_sub_ptr_       = std::make_shared<OdometrySubscriber>(nh_, "/synced_gnss", 100000);
    // pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, "/synced_cloud", 100000);
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, "/lslidar_point_cloud", 100000);
    loop_sub_ptr_       = std::make_shared<LoopSubscriber>(nh_, "/loop_pose", 100000);
    backend_pub_ptr_    = std::make_shared<OdometryPublisher>(nh_, backend_topic_name, "map", "lidar", 100);
    path_pub_ptr_       = std::make_shared<PathPublisher>(nh_, "/path", "map", 100);
    back_end_ptr_       = std::make_shared<BackEnd>();
    loop_detect_ptr_    = std::make_shared<LoopDetect>();

    use_gps_ = false;
}


bool BackEndFlow::run() {
    if (!readData())
        return false;
    
    while (hasData()) {
        if (!validData()) {
            // ROS_INFO("cloud:%.2f, odom:%.2f", cur_cloud_data_.time, cur_frontend_data_.time);
            // ros::Time time = ros::Time().fromSec(cur_cloud_data_.time);
            // ROS_INFO("cloud:%.2f", time.toSec());
            continue;
        }

        mayHaveLoop();

        if (updateGraph())
            publishData();
    }
    return true;
}


bool BackEndFlow::readData() {
    frontend_sub_ptr_->ParseData(frontend_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    pointcloud_sub_ptr_->parseData(cloud_data_buff_);
    loop_sub_ptr_->parseData(loop_data_buff_);
    return true;
}


bool BackEndFlow::hasData() {
    if (use_gps_ && gnss_data_buff_.empty())
        return false;
    return !(frontend_data_buff_.empty() || cloud_data_buff_.empty());
}


bool BackEndFlow::validData() {
    cur_frontend_data_ = frontend_data_buff_.front();
    cur_cloud_data_ = cloud_data_buff_.front();

    // 时间同步
    double diff_time_cloud= cur_frontend_data_.time - cur_cloud_data_.time;
    if (diff_time_cloud > 0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    if (diff_time_cloud < -0.05) {
        frontend_data_buff_.pop_front();
        return false;
    }

    // 使用GPS
    if (use_gps_) {
        cur_gnss_data_ = gnss_data_buff_.front();
        double diff_time_gnss = cur_frontend_data_.time - cur_gnss_data_.time;
        if (diff_time_gnss > 0.05) {
            gnss_data_buff_.pop_front();
            return false;
        }
        if (diff_time_gnss < -0.05 ) {
            frontend_data_buff_.pop_front();
            return false;
        }
        gnss_data_buff_.pop_front();
    }

    frontend_data_buff_.pop_front();
    cloud_data_buff_.pop_front();

    return true;
}


bool BackEndFlow::updateGraph() {
    if (use_gps_)
        return back_end_ptr_->update(cur_frontend_data_, cur_gnss_data_, cur_cloud_data_);
    else
        return back_end_ptr_->update(cur_frontend_data_, cur_cloud_data_);
}


bool BackEndFlow::publishData() {
    Eigen::Matrix4f optimized_pose;
    back_end_ptr_->getLatestOptimizedPose(optimized_pose);
    backend_pub_ptr_->publish(optimized_pose, cur_frontend_data_.time);

    std::deque<Eigen::Matrix4f> optimized_poses;
    back_end_ptr_->getOptimizedPoses(optimized_poses);
    path_pub_ptr_->publish(optimized_poses);

    return true;
}


bool BackEndFlow::mayHaveLoop() {
    while (!loop_data_buff_.empty()) {
        geometry_msgs::PoseWithCovarianceStamped pose_stamped = loop_data_buff_.front();
        Eigen::Matrix4f transform;
        transform(0, 3) = pose_stamped.pose.pose.position.x;
        transform(1, 3) = pose_stamped.pose.pose.position.y;
        transform(2, 3) = pose_stamped.pose.pose.position.z;
        Eigen::Quaternionf q;
        q.x() = pose_stamped.pose.pose.orientation.x;
        q.y() = pose_stamped.pose.pose.orientation.y;
        q.z() = pose_stamped.pose.pose.orientation.z;
        q.w() = pose_stamped.pose.pose.orientation.w;
        transform.block<3, 3>(0, 0) = q.matrix();
        back_end_ptr_->insertLoop(pose_stamped.pose.covariance.at(0), pose_stamped.pose.covariance.at(1), transform);
        loop_data_buff_.pop_front();

        ROS_INFO("Loop closed.");
    }
    return true;
}


bool BackEndFlow::forceOptimize() {
    return back_end_ptr_->forceOptimize();
}
