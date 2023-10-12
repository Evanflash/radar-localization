#include "radar_data.h"

namespace rawdata{

RadarData::RadarData(std::string &radar_data_path, std::string &radar_data_name)
    : cur_radar_data_timestamp(radar_data_name)
{
    std::string radar_data_file_path = radar_data_path + "/" + radar_data_name + "png";
    cv::Mat fft_data = cv::imread(radar_data_file_path, 0);
    cur_radar_data_row_theta.reserve(400);
    cur_radar_data_row_timestamp.reserve(400);
    cur_radar_data_raw.resize(400, std::vector<float>(4000, -1));
    
}

} // namespace rawdata

