#include <ros/ros.h>
#include "lidar_RTK/mapping/back_end/back_end_flow.hpp"
#include "lidar_RTK/optimizeMap.h"

bool _need_optimize_map = false;

bool optimize_map_callback(lidar_RTK::optimizeMap::Request &request, lidar_RTK::optimizeMap::Response &response) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->run();
        if (_need_optimize_map) {
            back_end_flow_ptr->forceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}
