#include "radar_sensor.hpp"
#include "radar_utils.hpp"

int main()
{
    radar::RadarSensor radar_sensor("/home/evan/code/radar-localization/test", 1547120953445209);
    radar_sensor.k_strongest_filter(40);
    radar_sensor.motion_compensation(radar::Vec3d(-0.19994, 0.972157, -0.148316));
    radar::CLOUD::Ptr n = radar_sensor.get_radar_point_cloud(radar::model::normal);
    radar::CLOUD::Ptr m = radar_sensor.get_radar_point_cloud(radar::model::motion);
    cv::Mat n_image = pointcloud_to_cartesian_points(n, 800, 800, 0.2);
    cv::Mat m_image = pointcloud_to_cartesian_points(m, 800, 800, 0.2);
    cv::Mat image(800, 1600, CV_8U);
    n_image.copyTo(image.colRange(0, 800));
    m_image.copyTo(image.colRange(800, 1600));
    cv::imshow("", image);
    cv::waitKey(0);
    return 0;
}