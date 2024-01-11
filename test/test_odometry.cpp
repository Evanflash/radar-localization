#include "odometry_fusion.hpp"

int main()
{
    odometry::Odometry odom("/home/evan/extra/datasets/large/radar", 
        "/home/evan/extra/datasets/large/radar.timestamps",
        "/home/evan/extra/datasets/large/gps/ins.csv");
    odom.laser_cloud_handler();
    return 0;
}