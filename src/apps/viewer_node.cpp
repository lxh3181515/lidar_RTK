#include <ros/ros.h>
#include "lidar_RTK/mapping/viewer/viewer_flow.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    std::shared_ptr<ViewerFlow> viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        viewer_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}
