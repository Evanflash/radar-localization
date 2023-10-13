#ifndef _RADAR_LOCALIZATION_K_STRONGEST_FILTER
#define _RADAR_LOCALIZATION_K_STRONGEST_FILTER

#include "base_filter.h"

namespace datafilter{

class KStrongestFilter : public BaseFilter{
public:
    KStrongestFilter(rawdata::RadarData &_radar_data);
    KStrongestFilter(rawdata::RadarData &_radar_data, int _k);
    void set_k(int _k);
    void filter() override;

private:
    int k;
}; // class KStrongestFilter 

} // namespcae datafilter

#endif // _RADAR_LOCALIZATION_K_STRONGEST_FILTER