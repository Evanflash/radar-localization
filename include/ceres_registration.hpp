#ifndef _RADAR_LOCALIZATION_CERES_REGISTRATION
#define _RADAR_LOCALIZATION_CERES_REGISTRATION

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ceres/loss_function.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<pcl::PointXYZI>;
using CloudTypePtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

std::vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources);

class RegistrationCost
{
protected:
    RegistrationCost() {}

    // 生成旋转矩阵
    template <typename T>
    static Eigen::Matrix<T, 2, 2> GetRotMatrix2D(const T* par)
    {
        const T cos_yaw = ceres::cos(par[2]);
        const T sin_yaw = ceres::sin(par[2]);
        Eigen::Matrix<T, 2, 2> rot;
        rot << cos_yaw, -sin_yaw,
               sin_yaw, cos_yaw;
        return rot;
    }
    // 生成平移矩阵
    template <typename T>
    static Eigen::Matrix<T, 2, 1> GetTransMatrix2D(const T* par)
    {
        Eigen::Matrix<T, 2, 1> trans{par[0], par[1]};
        return trans;
    }
}; // class RegistrationCost

class P2PCost : public RegistrationCost
{
public:
    P2PCost(const Vec2d target_mean, const Vec2d source_mean)
        : target_mean_(target_mean), source_mean_(source_mean) {}

    template<typename T>
    bool operator()(const T* Ta, const T* Tb, T* residuals_ptr) const
    {
        using Vec2T = Eigen::Matrix<T, 2, 1>;
        using Mat2T = Eigen::Matrix<T, 2, 2>;

        Vec2T target_trans = GetTransMatrix2D(Ta);
        Mat2T target_rot = GetRotMatrix2D(Ta);

        Vec2T source_trans = GetTransMatrix2D(Tb);
        Mat2T source_rot = GetRotMatrix2D(Tb);

        const Vec2T target_mean_transformed = (target_rot * (target_mean_).cast<T>()) + target_trans;
        const Vec2T source_mean_transformed = (source_rot * (source_mean_).cast<T>()) + source_trans;

        residuals_ptr[0] = target_mean_transformed(0) - source_mean_transformed(0);
        residuals_ptr[1] = target_mean_transformed(1) - source_mean_transformed(1);

        return true;
    }

    static ceres::CostFunction* Create(const Vec2d& target_mean, const Vec2d& source_mean)
    {
        return new ceres::AutoDiffCostFunction<P2PCost, 2, 3, 3>(new P2PCost(target_mean, source_mean));
    }

private:
    const Vec2d target_mean_;
    const Vec2d source_mean_;

}; // class P2PCost


#endif // _RADAR_LOCALIZATION_CERES_REGISTRATION