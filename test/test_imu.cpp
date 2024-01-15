#include <fstream>
#include <vector>
#include "imu_sensor.hpp"
#include "radar_utils.hpp"
int main()
{
    std::vector<long long> timestamps = read_timestamp_file("/home/evan/extra/datasets/large/radar.timestamps");
    long long target = timestamps[0];
    imu::IMUSensor imu_sensor("/home/evan/extra/datasets/large/gps/ins.csv");
    std::fstream output("/home/evan/code/radar-localization/test/result/imu.txt", std::ios::out);
    for(size_t i = 1; i < timestamps.size(); ++i){
        long long source = timestamps[i];
        Eigen::Vector3d pose = imu_sensor.get_relative_pose(target, source);
        output << source << " " << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
        target = source;
    }
    output.close();
    return 0;
}