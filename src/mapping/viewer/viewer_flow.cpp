#include "lidar_RTK/mapping/viewer/viewer_flow.hpp"


ViewerFlow::ViewerFlow(ros::NodeHandle& nh) :nh_(nh) {
    path_sub_ptr_ = std::make_shared<PathSubscriber>(nh_, "/path", 100000);
    map_pub_ptr_  = std::make_shared<PointcloudPublisher>(nh_, "/map", "map", 100);
    scan_pub_ptr_ = std::make_shared<PointcloudPublisher>(nh_, "/scan", "map", 100);
    viewer_ptr_   = std::make_shared<Viewer>();
}


bool ViewerFlow::run() {
    if (!readData())
        return false;
    
    while (hasData()) {
        if (!validData())
            continue;

        publishData();
    }
    return true;
}


bool ViewerFlow::readData() {
    path_sub_ptr_->ParseData(path_data_buff_);
    return true;
}


bool ViewerFlow::hasData() {
    return !path_data_buff_.empty();
}


bool ViewerFlow::validData() {
    std::vector<geometry_msgs::PoseStamped> poses;
    poses = path_data_buff_.front().poses;
    path_data_buff_.pop_front();

    optimized_poses_.clear();
    for (auto &pose:poses) {
        Eigen::Matrix4f matrix;
        matrix(0, 3) = pose.pose.position.x;
        matrix(1, 3) = pose.pose.position.y;
        matrix(2, 3) = pose.pose.position.z;
        Eigen::Quaternionf q;
        q.w() = pose.pose.orientation.w;
        q.x() = pose.pose.orientation.x;
        q.y() = pose.pose.orientation.y;
        q.z() = pose.pose.orientation.z;
        matrix.block<3, 3>(0, 0) = q.matrix();
        optimized_poses_.push_back(matrix);
    }
    return true;
}


bool ViewerFlow::publishData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>());

    static int frame_cnt = 0;

    // 间隔20个关键帧发布一次
    if (frame_cnt >= 20) {
        if (viewer_ptr_->getCurMap(optimized_poses_, cloud_ptr_))
            map_pub_ptr_->publish(cloud_ptr_);
        frame_cnt = 0;
    }
        
    if (frame_cnt % 2 == 1) {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (viewer_ptr_->getCurScan(optimized_poses_.size() - 1, optimized_poses_.back(), cloud_ptr_))
            scan_pub_ptr_->publish(cloud_ptr_);
    }

    frame_cnt++;

    return true;
}
