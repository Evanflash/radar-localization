#include <iostream>

#include "filter.h"
#include "data.h"
#include "show_point_cloud.h"

using namespace rawdata;
using namespace datafilter;
using namespace display;

int main()
{
    RadarData radar_data("/home/evan/code/radar-localization/test", "1547131046353776");
    BaseFilter *filter = new KStrongestFilter(radar_data, 12);
    filter -> filter();
    ShowPointCloud spc(radar_data.trans_to_point_cloud());
    spc.show(ShowPointCloud::IMAGE);
    return 0;
}