#ifndef _RADAR_LOCALIZATION_IMU_SENSOR
#define _RADAR_LOCALIZATION_IMU_SENSOR

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace imu
{

using ll = long long;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;

struct imu_data
{
    ll timestamp;
    float x_coor;
    float y_coor;
    float yaw;
    imu_data()
        : timestamp(0), x_coor(0), y_coor(0), yaw(0){}
    imu_data(ll timestamp_, float x_coor_, float y_coor_, float yaw_)
        : timestamp(timestamp_), x_coor(x_coor_), y_coor(y_coor_), yaw(yaw_){}
}; // struct imu_data

class IMUSensor
{
public:
    IMUSensor() = default;
    IMUSensor(const std::string imu_data_file_path);
    void search(ll timestamp, size_t &start, size_t &end);
    Vec3d get_imu_data_by_timestamp(ll timestamp);
    Vec3d get_relative_pose(Vec3d pre_pose_w, Vec3d nxt_pose_w);
    Vec3d get_relative_pose(ll pre_time, ll nxt_time);
    
private:
    Mat3d pose_to_transform(Vec3d pose);

private:
    std::vector<imu_data> all_imu_data;
}; // class IMUSensor

} // namespace imu

#endif // _RADAR_LOCALIZATION_IMU_SENSOR