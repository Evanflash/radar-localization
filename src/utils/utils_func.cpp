#include "utils_func.h"

#include "data.h"
#include "filter.h"

namespace utils{

Utils::CLOUDPTR Utils::radar_polar_to_point_cloud(const std::string &path, const std::string &name, filter_model fm)
{
    if(fm == Utils::kstrongest12){
        rawdata::RadarData radar_data(path, name);
        datafilter::BaseFilter *filter = new datafilter::KStrongestFilter(radar_data, 12);
        filter -> filter();
        return radar_data.trans_to_point_cloud();
    }
    return nullptr;
}

float Utils::sum(const std::vector<float> &nums, int begin, int end)
{
    float result = 0;
    for(int i = begin; i < end; ++i){
        result += nums[i];
    }
    return result;
}

} // namespace utils