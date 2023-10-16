#ifndef _RADAR_LOCALIZATION_BASE_FILTER
#define _RADAR_LOCALIZATION_BASE_FILTER

#include "radar_data.h"

namespace datafilter{

class BaseFilter{
public:
    BaseFilter(rawdata::RadarData &_radar_data);
    virtual void filter(){};

protected:
    rawdata::RadarData &radar_data;

}; // class BaseFilter

} // namespace datafilter

#endif // _RADAR_LOCALIZATION_BASE_FILTER