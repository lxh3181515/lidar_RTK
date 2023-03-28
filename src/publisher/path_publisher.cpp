#include "lidar_RTK/publisher/path_publisher.hpp"


PathPublisher::PathPublisher(ros::NodeHandle& nh, 
                            std::string topic_name, 
                            std::string base_frame_id,
                            int buff_size) 
    : nh_(nh){
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    path_.header.frame_id = base_frame_id;
}


void PathPublisher::publish(const std::deque<Eigen::Matrix4f>& transform_matrixs, double time) {
    publishData(transform_matrixs, ros::Time().fromSec(time));
}


void PathPublisher::publish(const std::deque<Eigen::Matrix4f>& transform_matrixs) {
    publishData(transform_matrixs, ros::Time::now());
}


void PathPublisher::publishData(const std::deque<Eigen::Matrix4f>& transform_matrixs, ros::Time time) {
    path_.header.stamp = time;
    path_.poses.clear();

    for (auto &matrix:transform_matrixs) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = matrix(0, 3);
        pose.pose.position.y = matrix(1, 3);
        pose.pose.position.z = matrix(2, 3);

        Eigen::Quaternionf q;
        q = matrix.block<3, 3>(0, 0);
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        path_.poses.push_back(pose);
    }

    publisher_.publish(path_);
}


bool PathPublisher::hasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
