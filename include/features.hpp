#ifndef _RADAR_LOCALIZATION_FEATURES
#define _RADAR_LOCALIZATION_FEATURES

#include <Eigen/Core>
#include <opencv2/core.hpp>

/** cen 2018文章中的特征提取
 * fft_data: fft结果
 * zq: 缩放门限
 * sigma_gauss: 高斯方差
 * min_range: 检测开始bin
 * targets: 特征点
 * return: 时间
*/
double cen_2018_features(cv::Mat fft_data, float zq, int sigma_gauss, int min_range, Eigen::MatrixXd &targets);

/** kstrongest特征提取算法
 * fft_data: fft结果
 * min_range: 检测开始bin
 * k: 每个方位角取k个值
 * targets: 特征点
 * return: 时间
*/
double k_strongest_features(cv::Mat fft_data, int min_range, int k, Eigen::MatrixXd &targets);

#endif // _RADAR_LOCALIZATION_FEATURES