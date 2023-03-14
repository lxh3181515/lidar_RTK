#include <ros/ros.h>
#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh, "/laser_odom", "/result");

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}
