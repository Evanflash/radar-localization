#include "gps_sensor.hpp"

#include <fstream>

namespace gps
{

GPSSensor::GPSSensor(const std::string file_path)
{
    std::fstream input_file(file_path.c_str(), std::ios::in);

    std::vector<Vec3d> gps_data_pose_raw;
    std::vector<ll> gps_data_timestamp_raw;

    std::string line;
    std::getline(input_file, line);
    while(std::getline(input_file, line)){
        std::stringstream ss(line);
        std::string str;
        std::vector<std::string> strs;
        strs.reserve(15);
        while(std::getline(ss, str, ',')){
            strs.push_back(str);
        }
        gps_data_timestamp_raw.push_back(std::stoll(strs[0]));
        gps_data_pose_raw.push_back(Vec3d(
            std::stod(strs[6]),
            std::stod(strs[5]),
            std::stod(strs[14])
        ));
        if(strs[1] == "INS_BAD_GPS_AGREEMENT"){
            bad_gps_value.insert(std::stoll(strs[0]));
        }
    }
    input_file.close();

    // 处理gps数据
    all_gps_data.push_back(gps_data(gps_data_timestamp_raw[0], 0, 0));
    double theta = -gps_data_pose_raw[0][2] + M_PI / 2;
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    for(size_t i = 1; i < gps_data_pose_raw.size(); ++i){
        double x = gps_data_pose_raw[i][0] - gps_data_pose_raw[0][0];
        double y = gps_data_pose_raw[i][1] - gps_data_pose_raw[0][1];
        all_gps_data.push_back(gps_data(
            gps_data_timestamp_raw[i],
            cos_theta * x + sin_theta * y,
            -sin_theta * x + cos_theta * y
        ));
    }
}


Vec2d GPSSensor::get_gps_data_by_timestamp(ll timestamp)
{
    bad_gps = false;
    size_t i = 0;
    size_t j = all_gps_data.size() - 1;
    search(timestamp, i, j);
    check_timestamp(all_gps_data[i].timestamp);
    check_timestamp(all_gps_data[j].timestamp);
    // 时间点
    if(i == j){
        gps_data cur = all_gps_data[i];
        return Vec2d(cur.x_coor, cur.y_coor);
    }
    // 时间段
    else{
        gps_data pre = all_gps_data[i];
        gps_data nxt = all_gps_data[j];
        float l1 = timestamp - pre.timestamp;
        float l2 = nxt.timestamp - timestamp;
        float w1 = l2 / (l1 + l2);
        float w2 = l1 / (l1 + l2);
        return Vec2d(w1 * pre.x_coor + w2 * nxt.x_coor,
                    w1 * pre.y_coor + w2 * nxt.y_coor);
    }
}

void GPSSensor::search(ll timestamp, size_t &start, size_t &end)
{
    if((end - start) == 1) return;

    size_t mid = (start + end) / 2;
    if(all_gps_data[mid].timestamp > timestamp){
        end = mid;
    }else if(all_gps_data[mid].timestamp < timestamp){
        start = mid;
    }else{
        start = mid;
        end = mid;
        return;
    }
    search(timestamp, start, end);
}

void GPSSensor::check_timestamp(ll timestamp)
{
    bad_gps = bad_gps || (bad_gps_value.find(timestamp) != bad_gps_value.end());
}

} // namespace gps