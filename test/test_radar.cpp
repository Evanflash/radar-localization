#include "radar_sensor.hpp"
#include "radar_utils.hpp"
#include "filter.hpp"
#include "imu_sensor.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ceres_registration.hpp"
#include "threshold.hpp"

using namespace std;
using ll = long long;
using Mat3d = Eigen::Matrix3d;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<pcl::PointXYZI>;
using CloudTypePtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using FeaturePoint = pcl::PointXY;

static string imu_data_file_path = "/home/evan/extra/datasets/large/gps/ins.csv";

enum model {normal, md, mdad, doppler};

struct feature
{
    FeaturePoint point;
    Eigen::Matrix<double, 1, 2> matD;
    Eigen::Matrix2d matV;
    feature(FeaturePoint _point, Eigen::Matrix<double, 1, 2> _matD, Eigen::Matrix2d _matV)
    {
        point = _point;
        matD = _matD;
        matV = _matV;
    }
};

Mat3d vec_to_transformation(Vec3d v)
{
    Eigen::Matrix3d T;
    double cos_t = cos(v[2]);
    double sin_t = sin(v[2]);
    T << cos_t, sin_t, v[0],
        -sin_t, cos_t, v[1],
        0, 0, 1;
    return T;
}

Mat3d vec_to_transformation(vector<double> v)
{
    Eigen::Matrix3d T;
    double cos_t = cos(v[2]);
    double sin_t = sin(v[2]);
    T << cos_t, sin_t, v[0],
        -sin_t, cos_t, v[1],
        0, 0, 1;
    return T;
}

void test_radar_timestamps(const string radar_file_path, const string save_file_path)
{
    fstream all_azimuth_timestamps_file(save_file_path.c_str(), std::ios::out);
    cv::Mat raw_data = cv::imread(radar_file_path, 0);
    for(int i = 0; i < raw_data.rows; ++i){
        int64 timestamp = 0;
        for(int j = 7; j >= 0; --j){
            int64 cur_num = (int64)raw_data.at<uchar>(i, j);
            timestamp = (timestamp << 8) + cur_num;
        }
        all_azimuth_timestamps_file << timestamp << endl;
    }
    all_azimuth_timestamps_file.close();
}

CloudTypePtr scan_denoise(const string radar_file_path, float contral, int range, model mdl)
{
    // 读取雷达图像数据
    vector<ll> radar_timestamps;
    vector<float> radar_azimuths;
    vector<vector<float>> radar_datas;

    cv::Mat raw_data = cv::imread(radar_file_path, 0);
    for(int i = 0; i < raw_data.rows; ++i)
    {
        static const float encode = M_PI / 2800.0;

        int64 timestamp = 0;
        for(int j = 7; j >= 0; --j)
        {
            int64 cur = (int64)raw_data.at<uchar>(i, j);
            timestamp = (timestamp << 8) + cur;
        }
        radar_timestamps.push_back(timestamp);
        
        uint16_t high_byte = (uint16_t)raw_data.at<uchar>(i, 9);
        uint16_t low_byte = (uint16_t)raw_data.at<uchar>(i, 8);
        float theta = (float)((high_byte << 8) + low_byte) * encode;
        radar_azimuths.push_back(theta);

        radar_datas.push_back(vector<float>());
        for(int j = 11; j < raw_data.cols; ++j)
        {
            float data = (float)raw_data.at<uchar>(i, j) / 255.0;
            radar_datas[i].push_back(data);
        }
    }

    // 运动去畸变
    Mat3d T_pre = Mat3d::Identity();
    vector<Mat3d> motion_distortion_vector{T_pre};
    imu::IMUSensor imu_sensor(imu_data_file_path);
    Vec3d T_all = imu_sensor.get_relative_pose(radar_timestamps.front(), radar_timestamps.back());
    for(uint i = 1; i < radar_timestamps.size(); ++i)
    {
        Mat3d T_cur = vec_to_transformation((i / 400.0) * T_all);
        motion_distortion_vector.push_back(T_cur);
    }
    // for(uint i = 1; i < radar_timestamps.size(); ++i)
    // {
    //     Mat3d T_cur = T_pre * vec_to_transformation(imu_sensor.get_relative_pose(radar_timestamps[i - 1], radar_timestamps[i]));
    //     motion_distortion_vector.push_back(T_cur);
    //     T_pre = T_cur;
    // }
    // 多普勒去畸变
    vector<vector<double>> doppler_offset_vector;
    for(uint i = 0; i < radar_timestamps.size() - 1; ++i)
    {
        Vec3d offset = imu_sensor.get_relative_pose(radar_timestamps[i], radar_timestamps[i + 1]);
        double time = (radar_timestamps[i + 1] - radar_timestamps[i]) * 0.000001;
        doppler_offset_vector.push_back(vector<double>{offset[0] / time, offset[1] / time});
    }
    doppler_offset_vector.push_back(doppler_offset_vector.back());

    vector<double> doppler_offset_dis;
    for(uint i = 0; i < radar_timestamps.size(); ++i)
    {
        double cos_t = cos(radar_azimuths[i]);
        double sin_t = sin(radar_azimuths[i]);
        doppler_offset_dis.push_back(0.0478 * 
            (doppler_offset_vector[i][0] * cos_t + doppler_offset_vector[i][1] * sin_t));
    }

    // 计算各个方位角上的均值，找到最小的均值作为阈值
    vector<float> radar_means;
    for(uint i = 0; i < radar_datas.size(); ++i)
    {
        float mean = 0;
        for(uint j = 0; j < radar_datas.back().size(); ++j)
            mean += radar_datas[i][j];
        mean /= radar_datas.back().size();
        radar_means.push_back(mean);
    }
    float threshold = 1000;
    for(uint i = 0; i < radar_means.size(); ++i)
    {
        threshold = min(threshold, radar_means[i]);
    }
    threshold *= contral;

    // 搜索每一列最大值的索引
    CloudTypePtr point_cloud_valid(new CloudType());
    const float resolution = 0.0438;
    for(uint i = 0; i < radar_datas.size(); i++)
    {
        // 获得变换矩阵
        Mat3d T;
        double d = 0;
        if(mdl == normal)
        {
            T = Mat3d::Identity();
        }else if(mdl == md)
        {
            T = motion_distortion_vector[i];
        }else if(mdl == mdad)
        {
            T = motion_distortion_vector[i];
            d = doppler_offset_dis[i];
        }else if(mdl == doppler)
        {
            T = Mat3d::Identity();
            d = doppler_offset_dis[i];
        }

        int index = 0;
        float value = 0;
        for(uint j = 0; j < radar_datas[i].size(); ++j){
            if(value < radar_datas[i][j])
            {
                index = j;
                value = radar_datas[i][j];
            }   
        }
        // 寻找index附近range范围内，且值大于threshold的点
        vector<int> valid_points;
        for(int ind = index; ind >= index - range && ind >= 0; --ind)
        {
            if(radar_datas[i][ind] > threshold + 2 * radar_means[i])
                valid_points.push_back(ind);
        }
        for(int ind = index + 1; ind <= index + range && ind < (int)radar_datas[i].size(); ++ind)
        {
            if(radar_datas[i][ind] > /*threshold +*/2 * radar_means[i])
                valid_points.push_back(ind);
        }
        // 转换到笛卡尔坐标系中
        for(auto ind : valid_points)
        {
            float distance = (ind + 0.5) * resolution + d;
            float azimuths = radar_azimuths[i];
            float cos_theta = cos(azimuths);
            float sin_theta = sin(azimuths);
            Vec3d p(distance * cos_theta, -distance * sin_theta, 1);
            p = T * p;
            PointType point(p[0], p[1], 0, 0);
            point_cloud_valid -> push_back(point);
        }
    }

    return point_cloud_valid;
}

// cv::Mat pointcloud_to_cartesian_points(CloudTypePtr point_cloud, int rows, int cols, float resolution)
// {
//     cv::Mat result = cv::Mat::zeros(rows, cols, CV_8U);
//     int dx = rows / 2;
//     int dy = cols / 2;
//     for(uint i = 0; i < point_cloud -> size(); ++i){
//         int x_ind = point_cloud -> points[i].x / resolution + dx;
//         int y_ind = point_cloud -> points[i].y / resolution + dy;
//         if(x_ind < 0 || x_ind >= rows || y_ind < 0 || y_ind >= cols) continue;
//         result.at<uchar>(x_ind, y_ind) = (uchar)(255);
//     }
//     return result;
// }

// 分割网格并计算方向和中心点
vector<feature> extract_features(CloudTypePtr cloud, float grid_resolution, int least_num)
{
    // 分割网格
    unordered_map<string, CloudType> grid;
    for(auto point : cloud -> points)
    {
        int x = point.x / grid_resolution;
        int y = point.y / grid_resolution;
        string key = to_string(x) + "_" + to_string(y);
        grid[key].push_back(point);
    }

    // 计算每个网格中的特征
    vector<feature> result;
    for(auto f : grid)
    {
        if(f.second.size() < 5) continue;

        FeaturePoint mean;
        Eigen::Matrix<double, 1, 2> matD = Eigen::Matrix<double, 1, 2>::Zero();
        Eigen::Matrix2d matV = Eigen::Matrix2d::Zero();

        int neighbor_num = f.second.size();
        for(auto point : f.second){
            mean.x += point.x;
            mean.y += point.y;
        }
        mean.x /= neighbor_num;
        mean.y /= neighbor_num;
        
        Eigen::Matrix2d matA = Eigen::Matrix2d::Zero();
        for(auto point : f.second){
            double ax = point.x - mean.x;
            double ay = point.y - mean.y;

            matA(0, 0) += ax * ax;
            matA(0, 1) += ax * ay;
            matA(1, 0) += ay * ax;
            matA(1, 1) += ay * ay;
        }
        matA /= neighbor_num;

        // 计算特征值
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> esolver(matA);
        matD = esolver.eigenvalues();
        matV = esolver.eigenvectors();

        result.push_back(feature(mean, matD, matV));
    }

    return result;
}

// pcl库中的icp算法
Eigen::Matrix4f pcl_icp_registration(CloudTypePtr source_cloud, CloudTypePtr target_cloud, int iterators)
{
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaximumIterations(iterators);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    CloudTypePtr output_cloud(new CloudType());
    icp.align(*output_cloud);
    
    return icp.getFinalTransformation();
}


// 测试每个方位角的时间戳
void test1()
{
    string radar_file_path = "/home/evan/code/radar-localization/test/1547131046353776.png";
    string save_file_path = "/home/evan/code/radar-localization/test/result/timestamps.txt";
    test_radar_timestamps(radar_file_path, save_file_path);
}

// 测试scan_denoise
void test2()
{
    string radar_file_path = "/home/evan/code/radar-localization/test/1547120953445209.png";
    CloudTypePtr cloud = scan_denoise(radar_file_path, 10, 5, normal);
    cv::Mat image = pointcloud_to_cartesian_points(cloud, 800, 800, 0.2);
    cv::imshow(to_string(cloud -> size()), image);
    cv::waitKey(0);
}

// 对比k strongest和scan denoise
void test3()
{
    radar::RadarSensor radar_sensor;
    radar_sensor.update_radar_data("/home/evan/code/radar-localization/test", 1547131046353776);

    radar_sensor.scan_denoise(3, 1);
    CloudTypePtr cloud1 = radar_sensor.get_radar_point_cloud(radar::normal);
    cv::Mat image1 = pointcloud_to_cartesian_points(cloud1, 800, 800, 0.2);

    radar_sensor.k_strongest_filter(12);
    CloudTypePtr cloud2 = radar_sensor.get_radar_point_cloud(radar::normal);
    cv::Mat image2 = pointcloud_to_cartesian_points(cloud2, 800, 800, 0.2);

    cv::Mat image(800, 1600, CV_8U);
    image1.copyTo(image.colRange(0, 800));
    image2.copyTo(image.colRange(800, 1600));

    CloudTypePtr cloud3(new CloudType());
    *cloud3 += *cloud1;
    *cloud3 += *cloud2;
    cv::Mat image3 = pointcloud_to_cartesian_points(cloud3, 800, 800, 0.2);

    cv::imshow("image", image);
    cv::waitKey(0);
}

// 显示提取的特征
void test4()
{
    string radar_file_path = "/home/evan/code/radar-localization/test/1547120953445209.png";
    CloudTypePtr cloud = scan_denoise(radar_file_path, 10, 5, normal);
    vector<feature> features = extract_features(cloud, 2, 5);
    CloudTypePtr feature_cloud(new CloudType());
    for(auto f : features)
    {
        PointType point(f.point.x, f.point.y, 0, 0);
        feature_cloud -> push_back(point);
    }

    cout << feature_cloud -> size() << endl;

    cv::Mat image1 = pointcloud_to_cartesian_points(cloud, 800, 800, 0.2);
    cv::Mat image2 = pointcloud_to_cartesian_points(feature_cloud, 800, 800, 0.2);

    cv::Mat image(800, 1600, CV_8U);
    image1.copyTo(image.colRange(0, 800));
    image2.copyTo(image.colRange(800, 1600));

    cv::imshow("image", image);
    cv::waitKey(0);
}

// 测试icp算法
void test5()
{
    string radar_file_path1 = "/home/evan/code/radar-localization/test/1547131046606586.png";
    CloudTypePtr cloud1 = scan_denoise(radar_file_path1, 10, 5, normal);
    vector<feature> features1 = extract_features(cloud1, 2, 5);
    CloudTypePtr feature_cloud1(new CloudType());
    for(auto f : features1)
    {
        PointType point(f.point.x, f.point.y, 0, 0);
        feature_cloud1 -> push_back(point);
    }
    
    string radar_file_path2 = "/home/evan/code/radar-localization/test/1547131046353776.png";
    CloudTypePtr cloud2 = scan_denoise(radar_file_path2, 10, 5, normal);
    vector<feature> features2 = extract_features(cloud2, 2, 5);
    CloudTypePtr feature_cloud2(new CloudType());
    for(auto f : features2)
    {
        PointType point(f.point.x, f.point.y, 0, 0);
        feature_cloud2 -> push_back(point);
    }
    
    Eigen::Matrix4f transform = pcl_icp_registration(cloud1, cloud2, 10);
    cout << transform << endl;

}

// 测试ceres配准函数
void test6()
{
    // string radar_file_path1 = "/home/evan/code/radar-localization/test/1547131046606586.png";
    // CloudTypePtr source_cloud = scan_denoise(radar_file_path1, 1, 5);

    // string radar_file_path2 = "/home/evan/code/radar-localization/test/1547131046353776.png";
    // CloudTypePtr target_cloud = scan_denoise(radar_file_path2, 1, 5);

    radar_data target_rd;
    radar_data_split("/home/evan/extra/datasets/large/radar", "1547131046353776", target_rd);
    CloudTypePtr target_cloud = k_strongest_filter(target_rd, 12, 0);
    radar_data source_rd;
    radar_data_split("/home/evan/extra/datasets/large/radar", "1547131046606586", source_rd);
    CloudTypePtr source_cloud = k_strongest_filter(source_rd, 12, 0);

    cv::Mat image = pointcloud_to_cartesian_points(source_cloud, 800, 800, 0.2);
    cv::imshow("", image);
    cv::waitKey(0);

    vector<double> result = P2PRegisterTest(target_cloud, source_cloud, vector<double>{0, 0, 0}, 2);

    cout << "x = " << result[0] << ", y = " << result[1] << ", yaw = " << result[2] << endl;
}

// 大数据量测试
void test7()
{
    const string save_path = 
    "/home/evan/code/radar-localization/test/result/normal_md_doppler_mdad/1_2_10/ceres_registration_md_other.txt";
    fstream output(save_path.c_str(), std::ios::out);
    vector<ll> timestamps = read_timestamp_file("/home/evan/extra/datasets/large/radar_change.timestamps");
    ll pre_timestamps = 0;
    ll cur_timestamps = 0;
    vector<double> pre_result = vector<double>{0, 0, 0};
    model mdl = md;
    imu::IMUSensor imu_sensor(imu_data_file_path);
    SearchThreshold search_threshold;
    for(uint i = 0; i < timestamps.size(); ++i)
    {
        cur_timestamps = timestamps[i];
        if(i > 0)
        {
            // scan denoise
            string target_file_path = "/home/evan/extra/datasets/large/radar/" + to_string(pre_timestamps) + ".png";
            CloudTypePtr target_cloud = scan_denoise(target_file_path, 1, 5, mdl);
            string source_file_path = "/home/evan/extra/datasets/large/radar/" + to_string(cur_timestamps) + ".png";
            CloudTypePtr source_cloud = scan_denoise(source_file_path, 1, 5, mdl);
            // radar_data target_rd;
            // radar_data_split("/home/evan/extra/datasets/large/radar", to_string(pre_timestamps), target_rd);
            // CloudTypePtr target_cloud = k_strongest_filter(target_rd, 12, 0);
            // radar_data source_rd;
            // radar_data_split("/home/evan/extra/datasets/large/radar", to_string(cur_timestamps), source_rd);
            // CloudTypePtr source_cloud = k_strongest_filter(source_rd, 12, 0);
            // vector<double> result = P2PRegisterTest(target_cloud, source_cloud, pre_result);
            double thres = search_threshold.computeThreshold();
            Vec3d t = imu_sensor.get_relative_pose(pre_timestamps, cur_timestamps);
            pre_result = vector<double>{t[0], t[1], t[2]};
            vector<double> result = P2PRegisterTest(target_cloud, source_cloud, pre_result, thres);
            output << to_string(cur_timestamps) << " " << result[0] << " " << result[1] << " " << result[2] << endl;
            
            search_threshold.updateDeltaT(vec_to_transformation(pre_result).inverse() * vec_to_transformation(result));
            
            pre_result = result;
            // std::cout << thres << std::endl;
        }
        pre_timestamps = cur_timestamps;
    }
    output.close();
}

void test8()
{
    string radar_file_path1 = "/home/evan/code/radar-localization/test/1547131046606586.png";

    radar_data rd;
    radar_data_split("/home/evan/code/radar-localization/test", to_string(1547131046606586), rd);
    CloudTypePtr cloud1 = k_strongest_filter(rd, 12, 0);
    // CloudTypePtr cloud1 = scan_denoise(radar_file_path1, 1, 2, normal);
    CloudTypePtr cloud2 = scan_denoise(radar_file_path1, 10, 5, mdad);

    cv::Mat image1 = pointcloud_to_cartesian_points(cloud1, 500, 500, 0.2);
    cv::Mat image2 = pointcloud_to_cartesian_points(cloud2, 500, 500, 0.2);

    cv::Mat image(500, 1000, CV_8U);
    image1.copyTo(image.colRange(0, 500));
    image2.copyTo(image.colRange(500, 1000));

    CloudTypePtr cloud3(new CloudType());
    *cloud3 += *cloud1;
    *cloud3 += *cloud2;
    cv::Mat image3 = pointcloud_to_cartesian_points(cloud3, 800, 800, 0.1);

    uint k_num = cloud1 -> size();
    uint s_num = cloud2 -> size();

    cv::imshow(std::to_string(k_num) + " _ " + std::to_string(s_num), image);
    cv::waitKey(0);

}

int main()
{
    auto ts = std::chrono::high_resolution_clock::now();
    test7();
    auto te = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = te - ts;
    std::cout << e.count() << std::endl;
    return 0;
}