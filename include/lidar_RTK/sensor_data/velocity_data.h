#ifndef LIDAR_RTK_SENSOR_DATA_VELOCITY_DATA_H_
#define LIDAR_RTK_SENSOR_DATA_VELOCITY_DATA_H_

#include <deque>
#include <Eigen/Dense>

class VelocityData {
public:
    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

public:
    static bool syncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
    void transformCoordinate(Eigen::Matrix4f transform_matrixf);
};

#endif
