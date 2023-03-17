#ifndef LIDAR_RTK_PUBLISHER_LOOP_PUBLISHER_HPP_
#define LIDAR_RTK_PUBLISHER_LOOP_PUBLISHER_HPP_

#include <ros/ros.h>
#include "Eigen/Dense"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


class LoopPublisher {
public:
    LoopPublisher(ros::NodeHandle& nh, 
                  std::string topic_name,
                  std::string frame_id_name,
                  int buff_size);

    void publish(const int old_index, 
                 const int new_index, 
                 const Eigen::Matrix4f& transform_matrix, 
                 double time);
    void publish(const int old_index, 
                 const int new_index, 
                 const Eigen::Matrix4f& transform_matrix);

private:
    void publishData(const int index, 
                     const int new_index, 
                     const Eigen::Matrix4f& transform_matrix, 
                     ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    geometry_msgs::PoseWithCovarianceStamped pose_stamped_;
    
};

#endif