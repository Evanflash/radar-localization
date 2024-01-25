#include "imu_sensor.hpp"

#include <fstream>
#include <sstream>

namespace imu
{

IMUSensor::IMUSensor(const std::string imu_data_file_path)
{
    std::fstream input_file(imu_data_file_path.c_str(), std::ios::in);

    float x_coor = 0;
    float y_coor = 0;

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
        x_coor += 0.02 * std::stof(strs[10]);
        y_coor += 0.02 * std::stof(strs[9]);
        this -> all_imu_data.push_back(imu_data(std::stoll(strs[0]), 
                                        x_coor, y_coor, std::stof(strs[14]) - M_PI / 2));
        if(strs[1] == "INS_BAD_GPS_AGREEMENT"){
            bad_gps_value.insert(std::stoll(strs[0]));
        }
    }

    input_file.close();
}

void IMUSensor::search(ll timestamp, size_t &start, size_t &end)
{
    if((end - start) == 1) return;

    size_t mid = (start + end) / 2;
    if(all_imu_data[mid].timestamp > timestamp){
        end = mid;
    }else if(all_imu_data[mid].timestamp < timestamp){
        start = mid;
    }else{
        start = mid;
        end = mid;
        return;
    }
    search(timestamp, start, end);
}

Vec3d IMUSensor::get_imu_data_by_timestamp(ll timestamp)
{
    size_t i = 0;
    size_t j = all_imu_data.size() - 1;
    search(timestamp, i, j);
    check_timestamp(all_imu_data[i].timestamp);
    check_timestamp(all_imu_data[j].timestamp);
    // 时间点
    if(i == j){
        imu_data cur = all_imu_data[i];
        return Vec3d(cur.x_coor, cur.y_coor, cur.yaw);
    }
    // 时间段
    else{
        imu_data pre = all_imu_data[i];
        imu_data nxt = all_imu_data[j];
        float l1 = timestamp - pre.timestamp;
        float l2 = nxt.timestamp - timestamp;
        float w1 = l2 / (l1 + l2);
        float w2 = l1 / (l1 + l2);
        return Vec3d(w1 * pre.x_coor + w2 * nxt.x_coor,
                    w1 * pre.y_coor + w2 * nxt.y_coor,
                    w1 * pre.yaw + w2 * pre.yaw);
    }
}

Vec3d IMUSensor::get_relative_pose(Vec3d pre_pose_w, Vec3d nxt_pose_w)
{
    Mat3d T_pre = pose_to_transform(pre_pose_w).inverse();
    Mat3d T_nxt = pose_to_transform(nxt_pose_w);
    Mat3d T = T_pre * T_nxt;
    double yaw = nxt_pose_w[2] - pre_pose_w[2];
    return Vec3d(T(0, 2), T(1, 2), yaw);
}
    
Vec3d IMUSensor::get_relative_pose(ll pre_time, ll nxt_time)
{
    bad_gps = false;
    Vec3d pre_pose_w = get_imu_data_by_timestamp(pre_time);
    Vec3d nxt_pose_w = get_imu_data_by_timestamp(nxt_time);
    return get_relative_pose(pre_pose_w, nxt_pose_w);
}

Mat3d IMUSensor::pose_to_transform(Vec3d pose)
{
    double sin_theta = sin(pose[2]);
    double cos_theta = cos(pose[2]);
    Mat3d T;
    T << cos_theta, sin_theta, pose[0],
        -sin_theta, cos_theta, pose[1],
        0, 0, 1;
    return T;
}

void IMUSensor::check_timestamp(ll timestamp)
{
    bad_gps = bad_gps || (bad_gps_value.find(timestamp) != bad_gps_value.end());
}

} // namespace imu