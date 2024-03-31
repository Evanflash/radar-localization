#ifndef _RADAR_LOCALIZATION_THRESHOLD
#define _RADAR_LOCALIZATION_THRESHOLD

#include <Eigen/Core>

using Mat3d = Eigen::Matrix3d;

class SearchThreshold
{
public:
    double computeModelError(Mat3d &delta_t);
    double computeThreshold();
    void updateDeltaT(Mat3d t)
    {
        delta_t_ = t;
    }

private:
    const double initial_threshold = 4.0;
    const double min_motion_th = 0.1;
    const double max_range = 100;

    double model_error_sse2 = 0;
    int num_samples = 0;

    Mat3d delta_t_ = Mat3d::Identity();
}; // class SearchThreshold

#endif // _RADAR_LOCALIZATION_THRESHOLD