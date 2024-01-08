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
}; // class Config

#endif // _RADAR_LOCALIZATION_ODOMETRY_CONFIG