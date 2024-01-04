#ifndef _RADAR_LOCALIZATION_RADAR_SENSOR
#define _RADAR_LOCALIZATION_RADAR_SENSOR

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar
{

using ll = long long;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;

enum model{
      normal,
      motion,
      doppler,
      motion_doppler,  
    };

class RadarSensor
{   
public:
    RadarSensor() = default;
    RadarSensor(const std::string radar_file_path, ll timestamp);
    void update_radar_data(const std::string radar_file_path, ll timestamp);
    void k_strongest_filter(int k);
    void motion_compensation(Vec3d relative_pose);
    CLOUD::Ptr get_radar_point_cloud(model md);

private:
    Mat3d pose_to_transformation(Vec3d pose);
    void transform_point(Mat3d &T, POINT &point);

private:
    ll timestamp;
    cv::Mat fft_data;
    std::vector<float> all_theta;
    std::vector<Mat3d> motion;
    std::vector<float> doppler;
    Eigen::MatrixXd targets;
}; // class RadarSensor

} // namespace radar

#endif //_RDARA_LOCALIZATION_RADAR_SENSOR