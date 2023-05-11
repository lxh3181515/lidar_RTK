#include "lidar_RTK/sensor_data/gnss_data.h"

#include <ros/ros.h>


double GNSSData::origin_longitude = 0.0;
double GNSSData::origin_latitude = 0.0;
double GNSSData::origin_altitude = 0.0;
bool GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian GNSSData::geo_converter;


void GNSSData::initOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);

    origin_longitude = longitude;
    origin_latitude = latitude;
    origin_altitude = altitude;

    origin_position_inited = true;
}


void GNSSData::updateXYZ() {
    if (!origin_position_inited) {
        ROS_WARN("GeoConverter has not set origin position");
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}   


bool GNSSData::syncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // Find two adjacent frames at sync_time
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) {
            // ROS_WARN("No former data:%.5f, %.5f", UnsyncedData.front().time, sync_time);
            return false;
        }
        if (UnsyncedData.at(1).time < sync_time) {
            // ROS_INFO("Pop front:%.5f, %.5f", UnsyncedData.at(1).time, sync_time);
            UnsyncedData.pop_front();
            continue;
        }
        // The interval between two frames is too long
        if (sync_time - UnsyncedData.front().time > 0.2 || UnsyncedData.at(1).time - sync_time > 0.2) {
            // ROS_WARN("GNSS data frames time error.");
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2) {
        // ROS_WARN("Not enough data.");
        return false;
    }
    
    // Linear interpolation
    GNSSData front_data = UnsyncedData.front();
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;
    float front_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    float back_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;
    synced_data.status  = back_data.status;
    synced_data.service = back_data.service;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude  = front_data.latitude  * front_scale + back_data.latitude  * back_scale;
    synced_data.altitude  = front_data.altitude  * front_scale + back_data.altitude  * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;
    SyncedData.push_back(synced_data);

    return true;
}
