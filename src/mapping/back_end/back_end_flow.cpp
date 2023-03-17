#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"


BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string frontend_topic_name, std::string backend_topic_name):nh_(nh) {
    frontend_sub_ptr_   = std::make_shared<OdometrySubscriber>(nh_, frontend_topic_name, 100000);
    gnss_sub_ptr_       = std::make_shared<OdometrySubscriber>(nh_, "/synced_gnss", 100000);
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, "/synced_cloud", 100000);
    backend_pub_ptr_    = std::make_shared<OdometryPublisher>(nh_, backend_topic_name, "map", "lidar", 100);
    traj_pub_ptr_       = std::make_shared<TrajectoryPublisher>(nh_, "/path", "map", 100);
    back_end_ptr_       = std::make_shared<BackEnd>();
    loop_detect_ptr_    = std::make_shared<LoopDetect>();
}


bool BackEndFlow::run() {
    if (!readData())
        return false;
    
    while (hasData()) {
        if (!validData())
            continue;

        if (updateGraph())
            publishData();

        if (loopDetect());

    }
    return true;
}


bool BackEndFlow::readData() {
    frontend_sub_ptr_->ParseData(frontend_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    pointcloud_sub_ptr_->parseData(cloud_data_buff_);
    return true;
}


bool BackEndFlow::hasData() {
    return !(frontend_data_buff_.empty() || gnss_data_buff_.empty() || cloud_data_buff_.empty());
}


bool BackEndFlow::validData() {
    cur_frontend_data_ = frontend_data_buff_.front();
    cur_gnss_data_ = gnss_data_buff_.front();
    cur_cloud_data_ = cloud_data_buff_.front();
    
    // 时间同步
    double diff_time_gnss = cur_frontend_data_.time - cur_gnss_data_.time;
    double diff_time_cloud= cur_frontend_data_.time - cur_cloud_data_.time;
    if (diff_time_gnss > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }
    if (diff_time_cloud > 0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    if (diff_time_gnss < -0.05 || diff_time_cloud < -0.05) {
        frontend_data_buff_.pop_front();
        return false;
    }
    
    frontend_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    cloud_data_buff_.pop_front();
    return true;
}


bool BackEndFlow::updateGraph() {
    return back_end_ptr_->update(cur_frontend_data_, cur_gnss_data_, cur_cloud_data_);
}


bool BackEndFlow::publishData() {
    back_end_ptr_->getLatestOptimizedPose(cur_optimized_pose_);
    back_end_ptr_->getOptimizedPoses(optimized_poses_);
    backend_pub_ptr_->publish(cur_optimized_pose_, cur_frontend_data_.time);
    traj_pub_ptr_->publish(optimized_poses_);
    return true;
}


bool BackEndFlow::loopDetect() {
    if (loop_detect_ptr_->update(optimized_poses_)) {
        int index;
        Eigen::Matrix4f transform;
        loop_detect_ptr_->getLoopTransMatrix(index, transform);
        back_end_ptr_->insertLoop(index, transform);
        ROS_INFO("Loop detected!");
        return true;
    }
    return false;
}


bool BackEndFlow::forceOptimize() {
    return back_end_ptr_->forceOptimize();
}
