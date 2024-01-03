#ifndef _RADAR_LOCALIZATION_RADAR_SENSOR
#define _RADAR_LOCALIZATION_RADAR_SENSOR

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar
{

using ll = long long;
using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;
using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;

class RadarSensor
{
public:
    enum model{
      normal,
      motion  
    };
public:
    RadarSensor(const std::string radar_file_path);
    void update_radar_data(const std::string radar_file_path);
    void k_strongest_filter(cv::Mat fft_data);
    void motion_compensation(Vec3d relative_pose);
    CLOUD::Ptr get_radar_point_cloud(model md);
private:
    ll timestamp;
    cv::Mat fft_data;
    std::vector<float> all_theta;
    std::vector<Vec2d> motion;
    Eigen::MatrixXd targets;
}; // class RadarSensor

} // namespace radar

#endif //_RDARA_LOCALIZATION_RADAR_SENSOR