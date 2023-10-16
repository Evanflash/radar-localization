#include <iostream>

#include "filter.h"
#include "data.h"
#include "show_point_cloud.h"
#include <thread>

using namespace rawdata;
using namespace datafilter;
using namespace display;

pcl::PointCloud<pcl::PointXYZI>::Ptr run(const std::string &path, const std::string &name)
{
    RadarData radar_data(path, name);
    // BaseFilter *filter = new KStrongestFilter(radar_data, 12);
    BaseFilter *filter = new CFARFilter(radar_data, CFARFilter::CA, 10, 0.1);
    filter -> filter();
    return radar_data.trans_to_point_cloud();
}

int main()
{
    ShowPointCloud::show(run("/home/evan/code/radar-localization/test", "1547131046353776"),
                    run("/home/evan/code/radar-localization/test", "1547131046606586"));
    // std::thread first(run, "/home/evan/code/radar-localization/test", "1547131046353776");
    // std::thread second(run, "/home/evan/code/radar-localization/test", "1547131046606586");
    // first.join();
    // second.join();
    return 0;
}