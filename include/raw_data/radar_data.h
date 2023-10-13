#ifndef _RADAR_LOCALIZATION_RADAR_DATA
#define _RADAR_LOCALIZATION_RADAR_DATA

#include <vector>
#include <string>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rawdata{

class RadarData{
private:
    using POINT = pcl::PointXYZI;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUDPTR = CLOUD::Ptr;
public:
    RadarData(std::string &radar_data_path, std::string &radar_data_name);
    CLOUDPTR trans_to_point_cloud();

private:
    int64 cur_radar_data_timestamp;                         // 文件时间戳
    std::vector<int64> cur_radar_data_row_timestamp;        // 每行数据时间戳
    std::vector<float> cur_radar_data_row_theta;            // 每行角度
    std::vector<std::vector<float>> cur_radar_data_raw;     // 原始幅度值
    std::vector<std::vector<bool>> cur_radar_data_flag;     // 标志对应的幅度值是否有效

}; // class RadarData

} // namespace rawdata

#endif // _RADAR_LOCALIZATION_RADAR_DATA