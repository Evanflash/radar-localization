#include "filter.hpp"

#include <iostream>
#include <vector>
#include <queue>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Eigen>


pcl::PointCloud<pcl::PointXYZI>::Ptr extract_surf_point(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, 
    int neighbor_points_num, float distance_max, float linear_max)
{
    struct linear{
        int index;
        float linear_value;
        linear(){}
        linear(int _index, float _linear_value)
            : index(_index), linear_value(_linear_value){}
    };

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_point_cloud(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_point_cloud -> setInputCloud(point_cloud);

    std::vector<linear> point_linear(point_cloud -> size());

    for(int i = 0; i < (int)point_cloud -> size(); ++i){
        std::vector<int> nn_idx(neighbor_points_num + 1);
        std::vector<float> nn_distance(neighbor_points_num + 1);
        kdtree_point_cloud -> nearestKSearch(point_cloud -> points[i], neighbor_points_num, nn_idx, nn_distance);
        if(nn_distance.back() > linear_max){
            point_linear[i] = linear(i, 0);
        } else{
            Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
            for(int i = 1; i <= neighbor_points_num; ++i){
                centroid += Eigen::Vector2f(point_cloud -> points[nn_idx[i]].x, point_cloud -> points[nn_idx[i]].y);
            }
            centroid /= neighbor_points_num;

            Eigen::Matrix2f covraiance = Eigen::Matrix2f::Zero();
            for(int i = 1; i <= neighbor_points_num; ++i){
                Eigen::Vector2f tmp = Eigen::Vector2f(point_cloud -> points[nn_idx[i]].x, 
                    point_cloud -> points[nn_idx[i]].y) - centroid;
                covraiance += tmp * tmp.transpose();
            }
            covraiance /= neighbor_points_num;
            Eigen::Vector2f eigenvalue = covraiance.eigenvalues().real();
            float value_larger = std::max(eigenvalue[0], eigenvalue[1]);
            float value_smaller = std::min(eigenvalue[0], eigenvalue[1]);
            point_linear[i] = linear(i, (value_larger - value_smaller) / (value_smaller + 0.001));
        }
    }

    for(int i = 0; i < (int) point_linear.size(); ++i){
        if(point_linear[i].linear_value > linear_max){
            result_cloud -> push_back(point_cloud -> points[point_linear[i].index]);
        }
    }
    // std::cout << result_cloud -> size() << std::endl;

    return result_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, float least_power)
{
    cv::Mat &fft_data = rd.fft_data; 
    std::vector<spectrum> &spectrum_vec = rd.spectrum_vec;

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < fft_data.rows; ++i){
        auto cmp = [&](const int l_val, const int r_val)
        {
            return fft_data.at<float>(i, l_val) > fft_data.at<float>(i, r_val);
        };

        float sin_theta = sin(spectrum_vec[i].theta);
        float cos_theta = cos(spectrum_vec[i].theta);

        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = 0; j < fft_data.cols; ++j){
            pri_que.push(j);
            while((int)pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            if(fft_data.at<float>(i, pri_que.top()) > least_power){
                float distance = (pri_que.top() + 0.5) * 0.0438;
                result_cloud -> push_back(pcl::PointXYZI(distance * sin_theta, distance * cos_theta, 
                    0, fft_data.at<float>(i, pri_que.top())));
            }
            pri_que.pop();
        }
    }

    pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxel_filter -> setInputCloud(result_cloud);
    voxel_filter -> setLeafSize(0.2, 0.2, 0.2);
    voxel_filter -> filter(*result_cloud);

    return result_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, 
    float least_power, Eigen::Vector3f init_pose)
{
    cv::Mat &fft_data = rd.fft_data; 
    std::vector<spectrum> &spectrum_vec = rd.spectrum_vec;

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < fft_data.rows; ++i){
        // doppler
        float delta_d = doppler_offset(init_pose[0], init_pose[1], spectrum_vec[i].theta);
        // float delta_d = 0;
        auto cmp = [&](const int l_val, const int r_val)
        {
            return fft_data.at<float>(i, l_val) > fft_data.at<float>(i, r_val);
        };

        float sin_theta = sin(spectrum_vec[i].theta);
        float cos_theta = cos(spectrum_vec[i].theta);

        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = 0; j < fft_data.cols; ++j){
            pri_que.push(j);
            while((int)pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            if(fft_data.at<float>(i, pri_que.top()) > least_power){
                float distance = (pri_que.top() + 0.5) * 0.0438 - delta_d;
                pcl::PointXYZI point_ori = pcl::PointXYZI(distance * sin_theta, distance * cos_theta, 
                    0, fft_data.at<float>(i, pri_que.top()));
                result_cloud -> push_back(motion_distortion(point_ori, i, init_pose[0], init_pose[1], init_pose[2]));
            }
            pri_que.pop();
        }
    }

    pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxel_filter -> setInputCloud(result_cloud);
    voxel_filter -> setLeafSize(0.2, 0.2, 0.2);
    voxel_filter -> filter(*result_cloud);

    return result_cloud;
}

float doppler_offset(float x_offset, float y_offset, float theta)
{
    float beta = 0.049;
    float delta_t = 0.25;
    float v_x = x_offset / delta_t;
    float v_y = y_offset / delta_t;
    float result = beta * (v_y * cos(theta) + v_x * sin(theta));
    return result;
}

pcl::PointXYZI motion_distortion(pcl::PointXYZI point_ori, int ind, float x_offset, float y_offset, float theta)
{
    int num = 400;
    float alpha = ind * theta / num;
    float x = ind * x_offset / num;
    float y = ind * y_offset / num;
    Eigen::Matrix3f T;
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);
    T << cos_alpha, sin_alpha, x,
        -sin_alpha, cos_alpha, y,
        0, 0, 1;
    Eigen::Vector3f p(point_ori.x, point_ori.y, 1);
    p = T * p;
    return pcl::PointXYZI(p[0], p[1], 0, point_ori.intensity);
}