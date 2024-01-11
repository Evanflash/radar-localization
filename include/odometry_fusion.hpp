#ifndef _RADAR_LOCALIZATION_ODOMETRY_FUSION
#define _RADAR_LOCALIZATION_ODOMETRY_FUSION

#include <string>
#include <vector>
#include <thread>
#include <fstream>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "imu_sensor.hpp"
#include "radar_sensor.hpp"
#include "odometry_config.hpp"
#include "radar_utils.hpp"

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
        const std::string imu_data_file_path);
    ~Odometry();

    void laser_cloud_handler();

    // 读取雷达时间戳文件
    void read_timestamps(const std::string radar_timestamp_file_path);

    // 获得初始相对位姿，用于运动补偿
    // 获得两关键帧之间的IMU因子
    Vec3d obtain_relative_pose(ll pre_timestamp, ll nxt_timestamp);
    
    // 提取当前帧周围的关键帧，组成targets
    void extract_surrounding_keyframes();
    void divide_into_grid(float grid_size, int least_points_num);
    void calculate_mean_and_cov(std::vector<POINT> point_set, Eigen::Vector2d& mean, 
        Eigen::Matrix<double, 1, 2>& matD, Eigen::Matrix2d& matV);
    // 单对多帧配置
    void scan_to_mulkeframes_optimization();
    // 判断是否为关键帧
    bool is_keyframes();
    // 保存关键帧并更新因子图
    void save_keyframes_and_factor();
    void add_odom_factor();
    void add_gps_factor();
    void add_loop_factor();

    void correct_poses();
    // 保存路径
    void save_path(const std::string save_file_path);

private:
    Mat3d pose_to_transformation(Vec3d pose);
    POINT transform_point(POINT point, Vec3d pose);
    POINT transform_point(POINT point, Mat3d pose);
    CLOUD::Ptr transform_cloud(CLOUD::Ptr cloud, Mat3d T);
    Vec3d relative_to_absolute_pose(Vec3d pose);
    gtsam::Pose2 vec_to_gtsam_pose(Vec3d pose);
private:
    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values initial_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isam_current_estimate;
    // 基本信息
    const std::string radar_file_path;
    std::vector<ll> timestamps;
    // 配置
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
    ll pre_timestamp;
    ll cur_timestamp;
    Vec3d cur_relative_pose; //当前帧与上一关键帧之间的相对位姿

    // 特征点及周围信息
    CLOUD::Ptr source_cloud;
    std::vector<std::vector<POINT>> source_feature_set;

    // 超参数
    int k;
    double keyframes_search_radius;
    ll save_keyframes_time_length;
    double save_keyframes_pose_dis;
    double save_keyframes_pose_yaw;
    int neighbor_num;
    int iterations;
    float grid_size;
    int least_point_num;
    std::string save_file_path;
};

} // namespace odometry

#endif //_RADAR_LOCALIZATION_ODOMETRY