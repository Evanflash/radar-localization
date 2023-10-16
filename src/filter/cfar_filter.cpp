#include "cfar_filter.h"
#include "utils_func.h"

namespace datafilter{

CFARFilter::CFARFilter(rawdata::RadarData &_radar_data, model _m, int _n, float _p)
    : BaseFilter(_radar_data), m(_m), n(_n), p(_p)
{
    T = std::pow(p, -0.5 / n) - 1;
}

void CFARFilter::filter()
{
    const std::vector<std::vector<float>> &radar_data_raw = radar_data.get_cur_radar_data_raw();
    std::vector<std::vector<bool>> &radar_data_flag = radar_data.get_cur_radar_data_flag();
    for(size_t i = 0; i < radar_data_raw.size(); ++i){
        for(size_t j = n; j < radar_data_raw.back().size() - 5; ++j){
            if(!radar_data_flag[i][j]) continue;
            float front_fft_sum = utils::Utils::sum(radar_data_raw[i], j - 5, j);
            float back_fft_sum = utils::Utils::sum(radar_data_raw[i], j + 1, j + 6);
            float Z = 0;
            if(m == CA) Z = (front_fft_sum + back_fft_sum);
            else if(m == GO) Z = (front_fft_sum > back_fft_sum ? front_fft_sum : back_fft_sum);
            else if(m == SO) Z = (front_fft_sum < back_fft_sum ? front_fft_sum : back_fft_sum);
            float value = Z * T;
            radar_data_flag[i][j] = (value < radar_data_raw[i][j]);
        }
    }
}



} // namespace datafilter