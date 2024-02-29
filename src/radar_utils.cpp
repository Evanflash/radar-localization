#include "radar_utils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

std::vector<std::string> features_model_string = {"kstrongest", 
                                                  "cen2018"};

std::vector<std::string> registration_model_string = {"icp"};


double radar_data_split(const std::string file_path, const std::string name, radar_data &rd)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    using ll = long long;
    ll timestamp = std::stoll(name);
    std::string radar_file_path = file_path + "/" + name + ".png";
    cv::Mat raw_data = cv::imread(radar_file_path, 0);
    cv::Mat fft_data = cv::Mat::zeros(raw_data.rows, raw_data.cols - 11, CV_32F);
    std::vector<spectrum> spectrum_vec(raw_data.rows, spectrum(0, 0));
    static const float encode = M_PI / 2800.0;
    for(int i = 0; i < raw_data.rows; ++i){
        for(int j = 0; j < 8; ++j){
            ll tmp = (ll) raw_data.at<uchar>(i, j);
            spectrum_vec[i].timestamp = (spectrum_vec[i].timestamp << 8) + tmp;
        }
        uint16_t high_byte = (uint16_t)raw_data.at<uchar>(i, 9);
        uint16_t low_byte = (uint16_t)raw_data.at<uchar>(i, 8);
        spectrum_vec[i].theta = (float)((high_byte << 8) + low_byte) * encode;
        for(int j = 11; j < raw_data.cols; ++j){
            fft_data.at<float>(i, j - 11) = (float)raw_data.at<uchar>(i, j) / 255.0;
        }
    }
    rd = radar_data(timestamp, spectrum_vec, fft_data);
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}

double targets_to_point_cloud(radar_data &rd, pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    point_cloud -> resize(rd.targets.cols());
    for(uint i = 0; i < rd.targets.cols(); ++i){
        double azimuth = rd.spectrum_vec[rd.targets(0, i)].theta;
        double distance = (rd.targets(1, i) + 0.5) * 0.0438;
        point_cloud -> points[i].x = distance * cos(azimuth);
        point_cloud -> points[i].y = distance * sin(azimuth);
        point_cloud -> points[i].intensity = 
            rd.fft_data.at<float>(rd.targets(0, i), rd.targets(1, i));
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}

cv::Mat targets_to_cartesian_points(radar_data &rd, int rows, int cols, float resolution)
{
    cv::Mat result = cv::Mat::zeros(rows, cols, CV_8U);
    int dx = rows / 2;
    int dy = cols / 2;
    for(uint i = 0; i < rd.targets.cols(); ++i){
        double azimuth = rd.spectrum_vec[rd.targets(0, i)].theta;
        double distance = (rd.targets(1, i) + 0.5) * 0.0438;
        int x_ind = distance * cos(azimuth) / resolution + dx;
        int y_ind = distance * sin(azimuth) / resolution + dy;
        if(x_ind < 0 || x_ind >= rows || y_ind < 0 || y_ind >= cols) continue;
        result.at<uchar>(x_ind, y_ind) = (uchar)(rd.fft_data.at<float>(rd.targets(0, i), rd.targets(1, i)) * 255);
    }
    return result;
}

cv::Mat pointcloud_to_cartesian_points(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, 
    int rows, int cols, float resolution)
{
    cv::Mat result = cv::Mat::zeros(rows, cols, CV_8U);
    int dx = rows / 2;
    int dy = cols / 2;
    for(uint i = 0; i < point_cloud -> size(); ++i){
        int x_ind = point_cloud -> points[i].x / resolution + dx;
        int y_ind = point_cloud -> points[i].y / resolution + dy;
        if(x_ind < 0 || x_ind >= rows || y_ind < 0 || y_ind >= cols) continue;
        result.at<uchar>(x_ind, y_ind) = (uchar)(255);
    }
    return result;
}


std::vector<long long> read_timestamp_file(const std::string &file_path)
{
    std::vector<long long> result;
    std::fstream input_timestamp_file(file_path.c_str(), std::ios::in);
    std::string line;
    while(std::getline(input_timestamp_file, line)){
        std::stringstream ss(line);
        std::string str;
        std::getline(ss, str, ' ');
        long long timestamp = std::stoll(str);
        result.push_back(timestamp);
    }

    input_timestamp_file.close();
    return result;
}

void save_transformation(const std::string &file_path, Eigen::Matrix4f pose, long long timestamp)
{
    std::fstream output_file(file_path.c_str(), std::ios::out | std::ios::app);
    output_file << timestamp << " " << 
        pose.block<3, 1>(0, 3)[0] << " " << pose.block<3, 1>(0, 3)[1] << " " << 
        pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0)[0] << std::endl;
    output_file.close();
}

std::vector<v_pose> read_gt_pose(const std::string file_path)
{
    std::vector<v_pose> gt_pose;
    std::fstream input_file(file_path.c_str(), std::ios::in);
    std::string line;
    getline(input_file, line);
    while(getline(input_file, line)){
        v_pose pose;
        std::stringstream ss(line);
        std::string str;
        getline(ss, str, ',');
        getline(ss, str, ',');

        getline(ss, str, ',');
        pose.x = std::stof(str);
        getline(ss, str, ',');
        pose.y = std::stof(str);

        getline(ss, str, ',');
        getline(ss, str, ',');
        getline(ss, str, ',');

        getline(ss, str, ',');
        pose.yaw = std::stof(str);

        getline(ss, str, ',');
        pose.timestamp = std::stoll(str);
        gt_pose.push_back(pose);
    }
    input_file.close();
    return gt_pose;
}

v_pose find_cloest_pose(std::vector<v_pose> &gt_pose, long long timestamp)
{
    v_pose pose = gt_pose.front();
    float min_num = std::abs(timestamp - pose.timestamp);
    for(uint i = 0; i < gt_pose.size(); ++i){
        if(std::abs(timestamp - gt_pose[i].timestamp) < min_num){
            min_num = std::abs(timestamp - gt_pose[i].timestamp);
            pose = gt_pose[i];
        }
    }
    return pose;
}

cv::Mat pointcloud_to_arrow_image(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, 
    int rows, int cols, float resolution)
{
    cv::Mat result = cv::Mat::zeros(rows, cols, CV_8U);
    int dx = rows / 2;
    int dy = cols / 2;
    for(uint i = 0; i < point_cloud -> size(); ++i){
        int x_ind = point_cloud -> points[i].x / resolution + dx;
        int y_ind = point_cloud -> points[i].y / resolution + dy;
        if(x_ind < 0 || x_ind >= rows || y_ind < 0 || y_ind >= cols) continue;
        result.at<uchar>(x_ind, y_ind) = (uchar)(255);

        int x_ind_end = (point_cloud -> points[i].x + 5 * point_cloud -> points[i].z) / resolution + dx;
        int y_ind_end = (point_cloud -> points[i].y + 5 * point_cloud -> points[i].intensity) / resolution + dy;
        if(x_ind_end < 0 || x_ind_end >= rows || y_ind_end < 0 || y_ind_end >= cols) continue;

        cv::arrowedLine(result, cv::Point(y_ind, x_ind), cv::Point(y_ind_end, x_ind_end),
            cv::Scalar(255), 1, 8, 0, 0.1);
    }
    return result;
}
