#include "registration.hpp"

Eigen::Matrix4f pcl_icp_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, int iterators)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterators);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    CloudT::Ptr output_cloud(new CloudT());
    icp.align(*output_cloud);

    return icp.getFinalTransformation();
}