#ifndef _RADAR_LOCALIZATION_SHOW_POINT_CLOUD
#define _RADAR_LOCALIZATION_SHOW_POINT_CLOUD

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"
#include "radar_data.h"

namespace display{

class ShowPointCloud{
public:
    enum model {IMAGE, POINT_CLOUD};
public:
    ShowPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud);
    void show(model m) const;
    static void show(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_1, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_2);
private:
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr point_cloud;
}; // class ShowPointCloud

} // namespace display

#endif // _RADAR_LOCALIZATION_SHOW_POINT_CLOUD