#ifndef _RADAR_LOCALIZATION_BASE_REGISTRATION
#define _RADAR_LOCALIZATION_BASE_REGISTRATION

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace registration{

using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec2d = Eigen::Matrix<double, 2, 1>;

class BaseRegistration
{
public:
    BaseRegistration() = default;
    virtual ~BaseRegistration() {};
    virtual bool registration(Sophus::SE2d &init_pose, int iterations_) { return true; };
    virtual void set_target(CLOUD::Ptr target_scan_){
        target_scan = target_scan_;
    }
    void set_source(CLOUD::Ptr source_scan_){
        source_scan = source_scan_;
    }

protected:
    CLOUD::Ptr target_scan = nullptr;
    CLOUD::Ptr source_scan = nullptr;
}; // class BaseRegistration

} // namespace registration

#endif // _RADAR_LOCALIZATION_BASE_REGISTRATION