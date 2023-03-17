#ifndef LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_FLOW_HPP_
#define LIDAR_RTK_MAPPING_LOOP_DETECT_LOOP_DETECT_FLOW_HPP_

#include "lidar_RTK/mapping/loop_detect/loop_detect.hpp"

class LoopDetectFlow {
public:
    LoopDetectFlow();

    bool run();
    bool readData();
    bool hasData();
    bool validData();
};

#endif