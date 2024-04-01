#include "threshold.hpp"

double SearchThreshold::computeModelError(Mat3d &delta_t)
{
    double delta_rot = 2 * max_range * sqrt((1 - delta_t(0, 0)) / 2);
    double delta_tran = sqrt(delta_t(0, 2) * delta_t(0, 2) + delta_t(1, 2) * delta_t(1, 2));
    return delta_rot + delta_tran;
}
double SearchThreshold::computeThreshold()
{
    double delta_error = computeModelError(delta_t_);
    if(delta_error > min_motion_th)
    {
        model_error_sse2 += delta_error * delta_error;
        num_samples++;
    }
    if(num_samples < 1)
    {
        return initial_threshold;
    }
    return std::sqrt(model_error_sse2 / num_samples);
}