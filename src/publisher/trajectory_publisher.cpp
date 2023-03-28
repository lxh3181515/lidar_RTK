#include "lidar_RTK/publisher/trajectory_publisher.hpp"


TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle& nh, 
                                       std::string topic_name, 
                                       std::string base_frame_id,
                                       int buff_size) 
    :nh_(nh), base_frame_id_(base_frame_id) {
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    path_.header.frame_id = base_frame_id_;
    path_.header.stamp = ros::Time::now();
}


void TrajectoryPublisher::publish(const std::deque<Eigen::Matrix4f> & transform_matrix, double time) {
    publishData(transform_matrix, ros::Time().fromSec(time));
}


void TrajectoryPublisher::publish(const std::deque<Eigen::Matrix4f> & transform_matrix) {
    publishData(transform_matrix, ros::Time::now());
}


void TrajectoryPublisher::publishData(const std::deque<Eigen::Matrix4f> & transform_matrix, ros::Time time) {
    path_.header.stamp = time;

    int size = transform_matrix.size();
    for (int i = 0; i < size; i++) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = base_frame_id_;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = transform_matrix.at(i)(0, 3);
        pose_stamped.pose.position.y = transform_matrix.at(i)(1, 3);
        pose_stamped.pose.position.z = transform_matrix.at(i)(2, 3);

        Eigen::Quaternionf q;
        q = transform_matrix.at(i).block<3, 3>(0, 0);
        pose_stamped.pose.orientation.w = q.w();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();

        path_.poses.push_back(pose_stamped);
    }

    publisher_.publish(path_);
}
