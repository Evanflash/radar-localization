#ifndef _RADAR_LOCALIZATION_CFAR_FILTER
#define _RADAR_LOCALIZATION_CFAR_FILTER

#include "base_filter.h"

namespace datafilter{

class CFARFilter : public BaseFilter
{
public:
    enum model {
        CA,
        GO,
        SO
    };
public:
    CFARFilter(rawdata::RadarData &_radar_data, model _m, int _n, float _p);
    void filter() override;

private:
    model m;            // 滤波模式
    int n;              // 保护单元个数
    float p;            // 虚警概率
    float T;            // 门限因子
}; // class CFARFilter

} // namespace datafilter

#endif // _RADAR_lOCALIZATION_CFAR_FILTER