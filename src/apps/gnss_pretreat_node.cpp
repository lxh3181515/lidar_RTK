#include <ros/ros.h>
#include "lidar_RTK/data_pretreat/gnss_pretreat_flow.hpp"


int main(int argc, char ** argv) {
    ros::init(argc, argv, "gnss_pretreat_node");
    ros::NodeHandle nh;

    std::shared_ptr<GNSSPretreatFlow> gnss_pretreat_flow_ptr = std::make_shared<GNSSPretreatFlow>(nh);

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        !gnss_pretreat_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}
