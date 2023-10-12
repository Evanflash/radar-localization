#ifndef _RADAR_LOCALIZATION_RADAR_DATA
#define _RADAR_LOCALIZATION_RADAR_DATA

#include <vector>
#include <string>
#include <fstream>

#include "opencv2/opencv.hpp"

namespace rawdata{

class RadarData{
public:
    RadarData(std::string &radar_data_path, std::string &radar_data_name);

private:
    const std::string cur_radar_data_timestamp;             // 文件时间戳
    std::vector<float> cur_radar_data_row_timestamp;        // 每行数据时间戳
    std::vector<float> cur_radar_data_row_theta;            // 每行角度
    std::vector<std::vector<float>> cur_radar_data_raw;     // 原始幅度值

}; // class RadarData

} // namespace rawdata

#endif // _RADAR_LOCALIZATION_RADAR_DATA