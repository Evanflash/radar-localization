#ifndef _RADAR_LOCALIZATION_BASE_REGISTRATION
#define _RADAR_LOCALIZATION_BASE_REGISTRATION

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace registration{

using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Vec3d = Eigen::Matrix<double, 3, 1>;

class SE2;
class BaseRegistration
{
public:
    virtual bool registration(SE2 &init_pose, int iterations_) { return true; };
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

class SE2
{
public:
    SE2() = default;
    SE2(float _x, float _y, float _theta);
    POINT operator*(POINT p);
    SE2& operator*(SE2 se2);
    float get_theta() const {return theta;}
    Mat3d get_trans() const;

    friend std::ostream& operator<<(std::ostream& os, const SE2& se2);

private:
    float x;
    float y;
    float theta;
}; // class SE2

} // namespace registration

#endif // _RADAR_LOCALIZATION_BASE_REGISTRATION