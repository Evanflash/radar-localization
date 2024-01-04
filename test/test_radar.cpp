#include "radar_sensor.hpp"
#include "radar_utils.hpp"

int main()
{
    radar::RadarSensor radar_sensor("/home/evan/code/radar-localization/test", 1547131050856549);
    radar_sensor.k_strongest_filter(40);
    radar::CLOUD::Ptr c = radar_sensor.get_radar_point_cloud(radar::model::normal);
    cv::Mat image = pointcloud_to_cartesian_points(c, 800, 800, 0.2);
    cv::imshow("", image);
    cv::waitKey(0);
    return 0;
}