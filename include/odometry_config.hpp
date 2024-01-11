#ifndef _RADAR_LOCALIZATION_ODOMETRY_CONFIG
#define _RADAR_LOCALIZATION_ODOMETRY_CONFIG

#include <string>

class Config
{
    using ll = long long;
public:
    Config(const std::string config_file_path);

public:
    int k;
    double keyframes_search_radius;
    ll save_keyframes_time_length;
    double save_keyframes_pose_dis;
    double save_keyframes_pose_yaw;
    int neighbor_num;
    int iterations;
    float grid_size;
    int least_point_num;
    std::string save_file_path;
}; // class Config

#endif // _RADAR_LOCALIZATION_ODOMETRY_CONFIG