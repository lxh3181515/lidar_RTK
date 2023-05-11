#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"
#include "lidar_RTK/global_defination/global_defination.h"


BackEndFlow::BackEndFlow(ros::NodeHandle& nh):nh_(nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/topic.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    frontend_sub_ptr_   = std::make_shared<OdometrySubscriber>(nh_, config_node["lidar_odom_topic"]["name"].as<std::string>(), 100000);
    gnss_sub_ptr_       = std::make_shared<OdometrySubscriber>(nh_, config_node["gnss_topic"].as<std::string>(), 100000);
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(nh_, config_node["pointcloud_topic"].as<std::string>(), 100000);
    loop_sub_ptr_       = std::make_shared<LoopSubscriber>(nh_, config_node["loop_topic"].as<std::string>(), 100000);
    
    backend_pub_ptr_    = std::make_shared<OdometryPublisher>(nh_, 
                                                              config_node["optimized_odom_topic"]["name"].as<std::string>(), 
                                                              config_node["optimized_odom_topic"]["base_frame"].as<std::string>(), 
                                                              config_node["optimized_odom_topic"]["child_frame"].as<std::string>(),
                                                              100);
    path_pub_ptr_       = std::make_shared<PathPublisher>(nh_, 
                                                          config_node["path_topic"]["name"].as<std::string>(), 
                                                          config_node["path_topic"]["base_frame"].as<std::string>(), 
                                                          100);
    scan_pub_ptr_       = std::make_shared<PointcloudPublisher>(nh_, 
                                                                config_node["scan_topic"]["name"].as<std::string>(), 
                                                                config_node["scan_topic"]["base_frame"].as<std::string>(), 
                                                                100);
    back_end_ptr_       = std::make_shared<BackEnd>();
    loop_detect_ptr_    = std::make_shared<LoopDetect>();

    use_gnss_ = config_node["use_gnss"].as<bool>();
    use_loop_close_ = config_node["use_loop_close"].as<bool>();
    have_gnss_ = false;
    valid_gnss_ = false;
    updated_ = false;

    optimized_pose_ = Eigen::Matrix4f::Identity();
}


bool BackEndFlow::run() {
    if (!readData())
        return false;
    
    while (hasData()) {
        if (!validData()) {
            continue;
        }

        mayHaveLoop();

        updateGraph();

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
    if (use_gnss_ && !gnss_data_buff_.empty())
        have_gnss_ = true;
    else
        have_gnss_ = false;
    
    return !(frontend_data_buff_.empty() || cloud_data_buff_.empty());
}


bool BackEndFlow::validData() {
    cur_frontend_data_ = frontend_data_buff_.front();
    cur_cloud_data_ = cloud_data_buff_.front();

    // 时间同步
    double diff_time_cloud = cur_frontend_data_.time - cur_cloud_data_.time;
    if (diff_time_cloud > 0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    if (diff_time_cloud < -0.05) {
        frontend_data_buff_.pop_front();
        return false;
    }

    frontend_data_buff_.pop_front();
    cloud_data_buff_.pop_front();

    // GPS
    static Eigen::Matrix4f gnss_init_pose;
    // static bool gnss_inited = false;
    if (have_gnss_) {
        double diff_time_gnss;

        cur_gnss_data_ = gnss_data_buff_.front();
        diff_time_gnss = cur_frontend_data_.time - cur_gnss_data_.time;

        while (diff_time_gnss > 0.05) {
            gnss_data_buff_.pop_front();
            if (gnss_data_buff_.empty())
                break;
            cur_gnss_data_ = gnss_data_buff_.front();
            diff_time_gnss = cur_frontend_data_.time - cur_gnss_data_.time;
        }

        if (diff_time_gnss < 0.05 && diff_time_gnss > -0.05) {
            // if (!gnss_inited) {
            //     gnss_inited = true;
            //     gnss_init_pose = cur_frontend_data_.pose * cur_gnss_data_.pose.inverse();
            // }
            // cur_gnss_data_.pose = gnss_init_pose * cur_gnss_data_.pose;
            valid_gnss_ = true;
            gnss_data_buff_.pop_front();
        }
    }

    return true;
}


bool BackEndFlow::updateGraph() {
    if (valid_gnss_) {
        valid_gnss_ = false;
        updated_ = back_end_ptr_->update(cur_frontend_data_, cur_gnss_data_, cur_cloud_data_);
    } else
        updated_ = back_end_ptr_->update(cur_frontend_data_, cur_cloud_data_);

    if (updated_) {
        std::deque<Eigen::Matrix4f> optimized_path;
        back_end_ptr_->getOptimizedPoses(optimized_path);
        path_pub_ptr_->publish(optimized_path);

        optimized_pose_ = optimized_path.back();
    }

    return updated_;
}


bool BackEndFlow::publishData() {
    static Eigen::Matrix4f last_frontend_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f delta_pose = last_frontend_pose.inverse() * cur_frontend_data_.pose;

    // Make sure in high freq
    if (!updated_) {
        optimized_pose_ = optimized_pose_ * delta_pose;
    }
        
    PointcloudData scan;
    pcl::transformPointCloud(*cur_cloud_data_.cloud_ptr, *scan.cloud_ptr, optimized_pose_);
    scan_pub_ptr_->publish(scan.cloud_ptr);
    backend_pub_ptr_->publish(optimized_pose_, cur_frontend_data_.time);

    last_frontend_pose = cur_frontend_data_.pose;

    return true;
}


bool BackEndFlow::mayHaveLoop() {
    if (!use_loop_close_) {
        loop_data_buff_.clear();
        return false;
    }

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
