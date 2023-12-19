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

Eigen::Vector3f point_to_line_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3f init_pose);

Eigen::Vector3f point_to_line_registration_weighted(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3f init_pose);

Eigen::Vector3f point_to_line_registration_weighted_mulkeyframe(
    CloudT::Ptr source_cloud, std::queue<CloudT::Ptr> target_clouds, Eigen::Vector3f init_pose);

Eigen::Vector3f common_P2L_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud,
    Eigen::Vector3f init_pose);

float huber_robust_core(float cost, float thres);

void calculate_mean_and_cov(std::vector<pcl::PointXYZI> point_set,
    Eigen::Vector2f& mean, Eigen::Matrix<float, 1, 2>& matD, Eigen::Matrix2f& matV);

#endif // _RADAR_LOCALIZATION_REGISTRATION