#ifndef _RADAR_LOCALIZATION_CLOUD_REGISTRATION
#define _RADAR_LOCALIZATION_CLOUD_REGISTRATION

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace registration
{

class CloudRegistration
{

using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;


public:
    Mat3d pose_to_transformation(Vec3d pose);
    POINT point_transform(POINT point, Vec3d pose);
}; // class CloudRegistration

} // namespace registration

#endif // _RADAR_LOCALIZATION_CLOUD_REGISTRATION