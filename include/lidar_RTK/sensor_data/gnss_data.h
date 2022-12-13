#ifndef LIDAR_RTK_SENSOR_DATA_GNSS_DATA_H_
#define LIDAR_RTK_SENSOR_DATA_GNSS_DATA_H_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"

class GNSSData {
public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

public:
    void initOriginPosition();
    void updateXYZ();
    static bool syncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};

#endif
