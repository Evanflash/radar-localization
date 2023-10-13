#include "radar_data.h"

namespace rawdata{

RadarData::RadarData(const std::string &radar_data_path, const std::string &radar_data_name)
{
    cur_radar_data_timestamp = std::stoll(radar_data_name);
    std::string radar_data_file_path = radar_data_path + "/" + radar_data_name + ".png";
    cv::Mat fft_data = cv::imread(radar_data_file_path, 0);
    cur_radar_data_row_theta.resize(fft_data.rows, 0);
    cur_radar_data_row_timestamp.resize(fft_data.rows, 0);
    cur_radar_data_raw.resize(fft_data.rows, std::vector<float>(fft_data.cols - 11, 0));
    cur_radar_data_flag.resize(fft_data.rows, std::vector<bool>(fft_data.cols - 11, true));
    static const float encode = M_PI / 2800.0;
    for(int i = 0; i < fft_data.rows; ++i){
        for(int j = 0; j < 8; ++j){
            int64 tmp = (int64) fft_data.at<uchar>(i, j);
            cur_radar_data_row_timestamp[i] = (cur_radar_data_row_timestamp[i] << 8) + tmp;
        }
        uint16_t high_byte = (uint16_t)fft_data.at<uchar>(i, 8);
        uint16_t low_byte = (uint16_t)fft_data.at<uchar>(i, 9);
        cur_radar_data_row_theta[i] = (float)((high_byte << 8) + low_byte);
        cur_radar_data_row_theta[i] *= encode;
        for(int j = 11; j < fft_data.cols; ++j){
            cur_radar_data_raw[i][j - 11] = (float)fft_data.at<uchar>(i, j) / 255.0;
        }
    }
}

RadarData::CLOUDPTR RadarData::trans_to_point_cloud()
{
    CLOUDPTR point_cloud(new CLOUD());
    point_cloud -> points.reserve(cur_radar_data_raw.size() * cur_radar_data_raw.back().size());
    for(size_t i = 0; i < cur_radar_data_row_theta.size(); ++i){
        float sin_theta = sin(cur_radar_data_row_theta[i]);
        float cos_theta = cos(cur_radar_data_row_theta[i]);
        for(size_t j = 0; j < cur_radar_data_raw.back().size(); ++j){
            if(cur_radar_data_flag[i][j]){
                float dis = (j + 1) * 0.0438;
                POINT point(dis * cos_theta, dis * sin_theta, 0.0, cur_radar_data_raw[i][j]);
                point_cloud -> push_back(point);
            }
        }
    }
    return point_cloud;
}

const int64& RadarData::get_cur_radar_data_timestamp() const
{
    return this -> cur_radar_data_timestamp;
}

const std::vector<int64>& RadarData::get_cur_radar_data_row_timestamp() const
{
    return this -> cur_radar_data_row_timestamp;
}

const std::vector<float>& RadarData::get_cur_radar_data_row_theta() const
{
    return this -> cur_radar_data_row_theta;
}

const std::vector<std::vector<float>>& RadarData::get_cur_radar_data_raw() const
{
    return this -> cur_radar_data_raw;
}

std::vector<std::vector<bool>>& RadarData::get_cur_radar_data_flag()
{
    return this -> cur_radar_data_flag;
}

} // namespace rawdata

