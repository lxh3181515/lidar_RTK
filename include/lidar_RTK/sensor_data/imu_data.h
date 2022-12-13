#ifndef LIDAR_RTK_SENSOR_DATA_IMU_DATA_H_
#define LIDAR_RTK_SENSOR_DATA_IMU_DATA_H_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

class IMUData {
public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    
    class Orientation {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    public:
        void normalize() {
            double norm = sqrt(x * x + y * y + z * z + w * w);
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };

    double time;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

public:
    Eigen::Matrix3f orientation2Matrix();
    static bool syncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};

#endif 
