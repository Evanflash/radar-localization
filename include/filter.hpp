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

#endif // _RADAR_LOCALIZATION_FILTER