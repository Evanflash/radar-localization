#ifndef _RADAR_LOCALIZATION_IMU_SENSOR
#define _RADAR_LOCALIZATION_IMU_SENSOR

#include <string>
#include <vector>
#include <unordered_set>

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
    double x_coor;
    double y_coor;
    double yaw;
    imu_data()
        : timestamp(0), x_coor(0), y_coor(0), yaw(0){}
    imu_data(ll timestamp_, double x_coor_, double y_coor_, double yaw_)
        : timestamp(timestamp_), x_coor(x_coor_), y_coor(y_coor_), yaw(yaw_){}
}; // struct imu_data

class IMUSensor
{
public:
    IMUSensor() = default;
    IMUSensor(const std::string imu_data_file_path);
    Vec3d get_relative_pose(Vec3d pre_pose_w, Vec3d nxt_pose_w);
    Vec3d get_relative_pose(ll pre_time, ll nxt_time);
    Vec3d get_imu_data_by_timestamp(ll timestamp);
    bool gps_data_is_bad(){
        return bad_gps;
    }
    
private:
    Mat3d pose_to_transform(Vec3d pose);
    void search(ll timestamp, size_t &start, size_t &end);
    void check_timestamp(ll timestamp);
    

private:
    std::vector<imu_data> all_imu_data;
    std::unordered_set<ll> bad_gps_value;
    bool bad_gps;

}; // class IMUSensor

} // namespace imu

#endif // _RADAR_LOCALIZATION_IMU_SENSOR