#include "lidar_RTK/publisher/loop_publisher.hpp"


LoopPublisher::LoopPublisher(ros::NodeHandle& nh, 
                             std::string topic_name, 
                             std::string frame_id_name,
                             int buff_size) 
    : nh_(nh) {
    publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
    pose_stamped_.header.frame_id = frame_id_name;
}


void LoopPublisher::publish(const int old_index, 
                            const int new_index, 
                            const Eigen::Matrix4f& transform_matrix, 
                            double time) {
    ros::Time ros_time((float)time);
    publishData(old_index, new_index, transform_matrix, ros_time);
}


void LoopPublisher::publish(const int old_index, 
                            const int new_index, 
                            const Eigen::Matrix4f& transform_matrix) {
    publishData(old_index, new_index, transform_matrix, ros::Time::now());
}


void LoopPublisher::publishData(const int old_index, 
                                const int new_index, 
                                const Eigen::Matrix4f& transform_matrix, 
                                ros::Time time) {
    pose_stamped_.header.stamp = time;

    pose_stamped_.pose.covariance.at(0) = old_index;
    pose_stamped_.pose.covariance.at(1) = new_index;

    pose_stamped_.pose.pose.position.x = transform_matrix(0, 3);
    pose_stamped_.pose.pose.position.y = transform_matrix(1, 3);
    pose_stamped_.pose.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3, 3>(0, 0);
    pose_stamped_.pose.pose.orientation.w = q.w();
    pose_stamped_.pose.pose.orientation.x = q.x();
    pose_stamped_.pose.pose.orientation.y = q.y();
    pose_stamped_.pose.pose.orientation.z = q.z();

    publisher_.publish(pose_stamped_);
}
