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

float Utils::theta(const float sin_t, const float cos_t)
{
    if(sin_t == 1.0) return 0.5 * M_PI;
    else if(sin_t == -1.0) return 1.5 * M_PI;
    else if(cos_t == 1.0) return 0;
    else if(cos_t == -1.0) return M_PI;

    float result = std::asin(sin_t);
    if(cos_t < 0) result = (result < 0) ? result - 0.5 * M_PI : result + 0.5 * M_PI;
    if(result < 0) result += 2 * M_PI;
    return result;
}

} // namespace utils