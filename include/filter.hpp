#ifndef _RADAR_LOCALIZATION_FILTER
#define _RADAR_LOCALIZATION_FILTER

#include "radar_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

/**
 * PCA
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr extract_surf_point(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, 
    int neighbor_points_num, float distance_max, float linear_max);

/**
 * k_strongest
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, float least_power);

/**
 * k_strongest with doppler and motion distortion
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, float least_power, Eigen::Vector3d init_pose);


/**
 * doppler
*/
float doppler_offset(float x_offset, float y_offset, float theta);

/**
 * motion distortion
*/
pcl::PointXYZI motion_distortion(pcl::PointXYZI point_ori, int ind, float x_offset, float y_offset, float theta);

/**
 * surf points extract
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr extract_flat_surf_points(
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, float grid_size);

/**
 * 
*/
std::vector<std::vector<pcl::PointXYZI>> divide_into_grid(
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, float grid_size, int least_points_num);

void cfar1d(cv::Mat fft_data, int window_size, float scale, int guard_cells, int min_range, Eigen::MatrixXd &targets);

/*!
   \brief Extract features from polar radar data using the method described in cen_icra18
   \param fft_data Polar radar power readings
   \param zq If y(i, j) > zq * sigma_q then it is considered a potential target point
   \param sigma_gauss std dev of the gaussian filter uesd to smooth the radar signal
   \param min_range We ignore the range bins less than this
   \param targets [out] Matrix of feature locations (azimuth_bin, range_bin, 1) x N
*/
double cen2018features(cv::Mat fft_data, float zq, int sigma_gauss, int min_range, Eigen::MatrixXd &targets);

/*!
   \brief Extract features from polar radar data using the method described in cen_icra19
   \param fft_data Polar radar power readings
   \param max_points Maximum number of targets points to be extracted from the radar image
   \param min_range We ignore the range bins less than this
   \param targets [out] Matrix of feature locations (azimuth_bin, range_bin, 1) x N
*/
double cen2019features(cv::Mat fft_data, int max_points, int min_range, Eigen::MatrixXd &targets);
#endif // _RADAR_LOCALIZATION_FILTER