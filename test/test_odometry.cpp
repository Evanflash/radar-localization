#include "odometry_fusion.hpp"

int main()
{
    // odometry::Odometry odom("/home/evan/extra/datasets/20190110-114621/radar", 
    //     "/home/evan/extra/datasets/20190110-114621/radar_change.timestamps",
    //     "/home/evan/extra/datasets/20190110-114621/gps/ins.csv",
    //     "/home/evan/extra/datasets/20190110-114621/gps/ins_change.csv");
    odometry::Odometry odom("/home/evan/extra/datasets/large/radar", 
    "/home/evan/extra/datasets/large/radar_change.timestamps",
    "/home/evan/extra/datasets/large/gps/ins.csv",
    "/home/evan/extra/datasets/large/gps/ins.csv");
    odom.laser_cloud_handler();
    return 0;
}