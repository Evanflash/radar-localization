#ifndef _RADAR_LOCALIZATION_ODOMETRY_FUSION
#define _RADAR_LOCALIZATION_ODOMETRY_FUSION

#include <string>
#include <vector>
#include <thread>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "imu_sensor.hpp"
#include "radar_sensor.hpp"
#include "odometry_config.hpp"

namespace odometry
{

using ll = long long;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;

class Odometry
{
public:
    Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path, const std::string config_file_path);
    ~Odometry();

    void laser_cloud_handler();

    // 读取雷达时间戳文件
    void read_timestamps(const std::string radar_timestamp_file_path);

    // 获得初始相对位姿，用于运动补偿
    // 获得两关键帧之间的IMU因子
    Vec3d obtain_relative_pose(ll pre_timestamp, ll nxt_timestamp);
    
    // 提取当前帧周围的关键帧，组成targets
    void extract_surrounding_keyframes();
    void scan_to_mulkeframes_optimization();
    // 判断是否为关键帧
    bool is_keyframes(ll cur_timestamp);
    
    void save_keyframes_and_factor(ll cur_timestamp);
    void correct_poses();

private:
    Mat3d pose_to_transformation(Vec3d pose);
    POINT transform_point(POINT point, Vec3d pose);
    POINT transform_point(POINT point, Mat3d pose);
    CLOUD::Ptr transform_cloud(CLOUD::Ptr cloud, Vec3d pose);
private:
    // 基本信息
    const std::string radar_file_path;
    std::vector<ll> timestamps;
    // 配置
    Config config;
    imu::IMUSensor imu_sensor;
    radar::RadarSensor radar_sensor;

    // 全局关键帧
    CLOUD::Ptr cloud_key_pose_2d;
    std::vector<CLOUD::Ptr> keyframe_clouds;
    std::vector<Vec3d> keyframe_poses;
    std::vector<ll> keyframe_timestamps;

    // targets关键帧
    std::vector<CLOUD::Ptr> surrounding_keyframe_clouds;

    // 位姿
    Vec3d pre_absolute_pose;
    Vec3d cur_relative_pose;
};

} // namespace odometry

#endif //_RADAR_LOCALIZATION_ODOMETRY