#ifndef _RADAR_LOCALIZATION_K_STRONGEST_FILTER
#define _RADAR_LOCALIZATION_K_STRONGEST_FILTER

#include "base_filter.h"

namespace datafilter{

class KStrongestFilter : public BaseFilter
{
public:
    enum filter_model {
        normal,
        threshold
    };
public:
    KStrongestFilter(rawdata::RadarData &_radar_data, int _k = 12, filter_model _fm = normal);
    void set_k(int _k);
    void filter() override;

private:
    int k;
    filter_model fm;
}; // class KStrongestFilter 

} // namespcae datafilter

#endif // _RADAR_LOCALIZATION_K_STRONGEST_FILTER