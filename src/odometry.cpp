#include "odometry.hpp"

namespace odometry
{

Odometry::Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path)
{
    read_timestamps(radar_timestamp_file_path);

}

Odometry::~Odometry()
{

}

} // namespace odometry