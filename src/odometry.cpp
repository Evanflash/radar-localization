#include "odometry.hpp"

namespace odometry
{

Odometry::Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path)
    : radar_file_path(radar_data_file_path), imu_sensor(imu_data_file_path)
{
    read_timestamps(radar_timestamp_file_path);
}

Odometry::~Odometry()
{

}

void Odometry::laser_cloud_handler()
{
    
}

void Odometry::read_timestamps(const std::string radar_timestamp_file_path)
{
    std::fstream input_file(radar_timestamp_file_path.c_str(), std::ios::in);
    std::string line;
    while(std::getline(input_file, line)){
        std::stringstream ss(line);
        std::string str;
        std::getline(ss, str, ' ');
        long long timestamp = std::stoll(str);
        timestamps.push_back(timestamp);
    }
    input_file.close();
}

} // namespace odometry