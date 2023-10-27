#ifndef _RADAR_LOCALIZATION_RADAR_UTILS
#define _RADAR_LOCALIZATION_RADAR_UTILS

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

enum features_model{
    kstrongest,
    cen2018
};
extern std::vector<std::string> features_model_string;

enum registration_model{
    icp
};
extern std::vector<std::string> registration_model_string;

struct spectrum
{
    using ll = long long;
    double theta;
    ll timestamp;
    spectrum(double _theta, ll _timestamp)
        : theta(_theta), timestamp(_timestamp){}
};

struct radar_data
{
    using ll = long long;
    ll timestamp;
    std::vector<spectrum> spectrum_vec;
    cv::Mat fft_data;
    Eigen::MatrixXd targets;
    radar_data(){}
    radar_data(ll _timestamp, std::vector<spectrum> _spectrum_vec, cv::Mat _fft_data)
        : timestamp(_timestamp), spectrum_vec(_spectrum_vec), fft_data(_fft_data){}
    radar_data(ll _timestamp, std::vector<spectrum> _spectrum_vec, cv::Mat _fft_data, Eigen::MatrixXd _targets)
        : timestamp(_timestamp), spectrum_vec(_spectrum_vec), fft_data(_fft_data), targets(_targets){}
    void set_targets(Eigen::MatrixXd _targets){
        targets = _targets;
    }
};


/** 将雷达图像拆解成spectrum和fft_data
 * file_path: 文件位置
 * name: 文件名称
 * rd: radar_data
 * return: 时间
*/
double radar_data_split(const std::string file_path, const std::string name, radar_data &rd);

/** 将目标点转换为点云
 * rd: radar_data, targets需要有值
 * point_cloud: 点云
 * return: 时间
*/
double targets_to_point_cloud(radar_data &rd, pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);

/** 将目标点转换为笛卡尔坐标
 * rd: 雷达数据
 * rows: 结果的行数
 * cols: 结果的列数
 * resolution: 分辨率
 * return: 转换结果
*/
cv::Mat targets_to_cartesian_points(radar_data &rd, int rows, int cols, float resolution);


/**
 * 读取时间戳文件
*/
std::vector<long long> read_timestamp_file(const std::string &file_path);

/**
 * 保存变换矩阵
*/
void save_transformation(const std::string &file_path, Eigen::Matrix4f pose, long long timestamp);


#endif // _RADAR_LOCALIZATION_RADAR_UTILS