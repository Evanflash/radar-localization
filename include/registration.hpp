#ifndef _RADAR_LOCALIZATION_REGISTRATION
#define _RADAR_LOCALIZATION_REGISTRATION

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <queue>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Eigen::Matrix4f pcl_icp_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, int iterators);

Eigen::Vector3d point_to_line_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3d init_pose);

Eigen::Vector3d point_to_line_registration_weighted(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3d init_pose);

Eigen::Vector3d point_to_line_registration_weighted_mulkeyframe(
    CloudT::Ptr source_cloud, std::queue<CloudT::Ptr> target_clouds, Eigen::Vector3d init_pose);

Eigen::Vector3d common_P2L_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud,
    Eigen::Vector3d init_pose);

double huber_robust_core(double cost, double thres);

void calculate_mean_and_cov(std::vector<pcl::PointXYZI> point_set,
    Eigen::Vector2d& mean, Eigen::Matrix<double, 1, 2>& matD, Eigen::Matrix2d& matV);

Eigen::Vector3d point_to_line_registration_weighted_mulkeyframe_cauchy(
    CloudT::Ptr source_cloud, std::queue<CloudT::Ptr> target_clouds, Eigen::Vector3d init_pose);

Eigen::Vector3d point_to_line_registration_weighted_mulkeyframe_ceres(
    CloudT::Ptr source_cloud, std::queue<CloudT::Ptr> target_clouds, Eigen::Vector3d init_pose);


#endif // _RADAR_LOCALIZATION_REGISTRATION