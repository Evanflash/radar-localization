#include "imu_sensor.hpp"

#include <fstream>
#include <sstream>

#include <Eigen/Geometry>

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
        x_coor += 0.02 * std::stod(strs[10]);
        y_coor += 0.02 * std::stod(strs[9]);
        this -> all_imu_data.push_back(imu_data(std::stoll(strs[0]), 
                                        x_coor, y_coor, std::stod(strs[14]) - M_PI / 2));
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
        double alpha = (timestamp - pre.timestamp) / (nxt.timestamp - pre.timestamp);
        Eigen::Matrix3d R;
        R = 
            Eigen::AngleAxisd(pre.yaw, Eigen::Vector3d::UnitZ()) * 
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond rot1(R);
        R = 
            Eigen::AngleAxisd(nxt.yaw, Eigen::Vector3d::UnitZ()) * 
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond rot2(R);
        R = rot1.slerp(alpha, rot2).toRotationMatrix();
        return Vec3d((1 - alpha) * pre.x_coor + alpha * nxt.x_coor,
                    (1 - alpha) * pre.y_coor + alpha * nxt.y_coor,
                    R.eulerAngles(0, 1, 2)(2));
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