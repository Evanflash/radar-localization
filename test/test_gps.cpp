#include <fstream>
#include <vector>
#include "gps_sensor.hpp"
#include "radar_utils.hpp"
int main()
{
    std::vector<long long> timestamps = read_timestamp_file("/home/evan/extra/datasets/20190110-114621/radar_change.timestamps");
    gps::GPSSensor gps_sensor("/home/evan/extra/datasets/20190110-114621/gps/ins_change.csv");
    std::fstream output("/home/evan/code/radar-localization/test/result/gps.txt", std::ios::out);
    for(size_t i = 0; i < timestamps.size(); ++i){
        long long source = timestamps[i];
        Eigen::Vector2d pose = gps_sensor.get_gps_data_by_timestamp(source);
        output << source << " " << pose[0] << " " << pose[1] << " " << 0 << std::endl;
    }
    output.close();
    return 0;
}