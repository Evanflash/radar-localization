#ifndef _RADAR_LOCALIZATION_ODOMETRY
#define _RADAR_LOCALIZATION_ODOMETRY

#include <string>
#include <vector>
#include <thread>
#include <fstream>

#include "imu_sensor.hpp"
#include "radar_sensor.hpp"

namespace odometry
{

using ll = long long;

class Odometry
{
public:
    Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path);
    ~Odometry();

    void read_timestamps(const std::string radar_timestamp_file_path);
    void laser_cloud_handler();
    void update_initial_guess();
    void extract_surrounding_keyframes();
    void scan_to_mulkeframes_optimization();
    void save_keyframes_and_factor();
    void correct_poses();

private:
    const std::string radar_file_path;
    std::vector<ll> timestamps;

    imu::IMUSensor imu_sensor;
    radar::RadarSensor radar_sensor;
};

} // namespace odometry

#endif //_RADAR_LOCALIZATION_ODOMETRY