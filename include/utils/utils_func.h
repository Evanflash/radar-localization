#ifndef _RADAR_LOCALIZATION_UTILS_FUNC
#define _RADAR_LOCALIZATION_UTILS_FUNC

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace utils{

class Utils{
public:
    using POINT = pcl::PointXYZI;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUDPTR = CLOUD::Ptr;
    using CLOUDCPTR = CLOUD::ConstPtr;

    enum filter_model {
        kstrongest12
        };

public:
    static CLOUDPTR radar_polar_to_point_cloud(const std::string &path, const std::string &name, filter_model fm);
    static float sum(const std::vector<float> &nums, int begin, int end);
    static float theta(const float sin_t, const float cos_t);
}; // class Utils

} // namespace utils

#endif // _RADAR_LOCALIZATION_UTILS_FUNC