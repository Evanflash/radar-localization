#include <iostream>

#include "filter.h"
#include "data.h"

using namespace rawdata;
using namespace datafilter;

int main()
{
    RadarData radar_data("/home/evan/code/radar-localization/test", "1547131046353776");
    BaseFilter *filter = new KStrongestFilter(radar_data, 2);
    filter -> filter();
    return 0;
}