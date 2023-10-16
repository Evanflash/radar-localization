#include "show_point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"

namespace display{

ShowPointCloud::ShowPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud)
    : point_cloud(_point_cloud){}


void ShowPointCloud::show(model m) const
{
    if(m == IMAGE){
        static float d = 0.2;
        static int dx = 400;
        static int dy = 400;
        cv::Mat image(800, 800, CV_8U, cv::Scalar(0));
        for(auto point : point_cloud -> points){
            int cur_x = point.x / d;
            int cur_y = point.y / d;
            cur_x += dx;
            cur_y += dy;
            if(cur_x < 0 || cur_x >= 800 || cur_y < 0 || cur_y >= 800) continue;
            image.at<uchar>(cur_x, cur_y) = (uchar)(point.intensity * 255.0);
        }
        cv::imshow("image", image);
        cv::waitKey(0);
    } else if(m == POINT_CLOUD){
        pcl::visualization::PCLVisualizer pcl_visualizer("point_cloud|size:" + std::to_string(point_cloud -> size()));
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red(point_cloud, 255, 0, 0);
        // pcl_visualizer.addPointCloud(point_cloud, red);
        pcl_visualizer.addPointCloud<pcl::PointXYZI>(point_cloud);
        pcl_visualizer.spin();
    }
}

} // namespace display