#include <ros/ros.h>
#include "lidar_RTK/mapping/front_end/front_end_flow.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}
