#include "k_strongest_filter.h"
#include "utils_func.h"
#include <queue>

namespace datafilter{

KStrongestFilter::KStrongestFilter(rawdata::RadarData &_radar_data, int _k, filter_model _fm)
    : BaseFilter(_radar_data), k(_k), fm(_fm){}

void KStrongestFilter::set_k(int _k)
{
    this -> k = _k;
}

void KStrongestFilter::filter()
{
    const std::vector<std::vector<float>> &radar_data_raw = radar_data.get_cur_radar_data_raw();
    std::vector<std::vector<bool>> &radar_data_flag = radar_data.get_cur_radar_data_flag();
    
    if(fm == threshold){
        for(size_t i = 0; i < radar_data_raw.size(); ++i){
            int useful_num = 0;
            for(size_t j = 0; j < radar_data_raw[i].size(); ++j){
                if(radar_data_raw[i][j] > 0) useful_num++;
            }
            float thred = utils::Utils::sum(radar_data_raw[i], 0, radar_data_raw[i].size()) / useful_num;
            for(size_t j = 0; j < radar_data_raw.back().size(); ++j){
                radar_data_flag[i][j] = radar_data_raw[i][j] > thred;
            }
        }
    }

    for(size_t i = 0; i < radar_data_raw.size(); ++i){
        auto cmp = [&](const size_t &l_index, const size_t &r_index)
        {
            return radar_data_raw[i][l_index] > radar_data_raw[i][r_index];
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(size_t j = 0; j < radar_data_raw.back().size(); ++j){
            if(!radar_data_flag[i][j]) continue;
            pri_que.push(j);
            while(pri_que.size() > (size_t)k){
                pri_que.pop();
            }
        }
        // 标记
        radar_data_flag[i].assign(radar_data_flag.back().size(), false);
        while(!pri_que.empty()){
            radar_data_flag[i][pri_que.top()] = true;
            pri_que.pop();
        }
    }
}

} // namespcae datafilter