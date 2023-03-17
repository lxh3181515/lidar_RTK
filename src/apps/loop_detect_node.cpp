#include <ros/ros.h>
#include "lidar_RTK/mapping/loop_detect/loop_detect_flow.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "loop_detect_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopDetectFlow> loop_detect_flow_ptr = std::make_shared<LoopDetectFlow>(nh, "/optimized_pose");

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        loop_detect_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}
