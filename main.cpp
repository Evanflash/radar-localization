#include <iostream>

#include "filter.h"
#include "data.h"
#include "show_point_cloud.h"
#include "utils_func.h"
#include <thread>

using namespace rawdata;
using namespace datafilter;
using namespace display;
using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr run(const std::string &path, const std::string &name)
{
    RadarData radar_data(path, name);
    BaseFilter *filter = new KStrongestFilter(radar_data, 40, KStrongestFilter::threshold);
    // BaseFilter *filter = new CFARFilter(radar_data, CFARFilter::CA, 5, 1e-6);
    filter -> filter();
    return radar_data.trans_to_point_cloud();
}

int main()
{
    // ShowPointCloud::show(run("/home/evan/code/radar-localization/test", "1547131046353776"),
    //                 run("/home/evan/code/radar-localization/test", "1547131046606586"));
    cout << utils::Utils::theta(0.707, 0.707) << endl;
    cout << utils::Utils::theta(0.707, -0.707) << endl;
    cout << utils::Utils::theta(-0.707, -0.707) << endl;
    cout << utils::Utils::theta(-0.707, 0.707) << endl;
    return 0;
}