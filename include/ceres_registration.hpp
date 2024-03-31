#ifndef _RADAR_LOCALIZATION_CERES_REGISTRATION
#define _RADAR_LOCALIZATION_CERES_REGISTRATION

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ceres/loss_function.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>

#include "normal_feature.hpp"

using std::vector;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<pcl::PointXYZI>;
using CloudTypePtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

std::vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources, std::vector<double> init_pose, double thres);
std::vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources, std::vector<double> init_pose, double thres, double rsize, double rsearch);
std::vector<double> P2PMulKeyFrameRegisterTest(vector<MapFeatures> clouds_, vector<vector<double>> transforms_, double thres);
std::vector<double> P2PMulKeyFrameRegisterInit(vector<MapFeatures> clouds_, vector<vector<double>> transforms_, double thres);

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
        rot << cos_yaw, sin_yaw,
               -sin_yaw, cos_yaw;
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


class P2PMulKeyFrameCost : public RegistrationCost
{
public:
    P2PMulKeyFrameCost(const Vec2d target_mean, const Vec2d source_mean, const double weight)
        : target_mean_(target_mean), source_mean_(source_mean), weight_(weight) {}

    template<typename T>
    bool operator()(const T* Tb, T* residuals_ptr) const
    {
        using Vec2T = Eigen::Matrix<T, 2, 1>;
        using Mat2T = Eigen::Matrix<T, 2, 2>;

        Vec2T source_trans = GetTransMatrix2D(Tb);
        Mat2T source_rot = GetRotMatrix2D(Tb);

        const Vec2T target_mean_transformed = (target_mean_).cast<T>();
        const Vec2T source_mean_transformed = (source_rot * (source_mean_).cast<T>()) + source_trans;

        residuals_ptr[0] = (target_mean_transformed(0) - source_mean_transformed(0)) * (T)weight_;
        residuals_ptr[1] = (target_mean_transformed(1) - source_mean_transformed(1)) * (T)weight_;

        return true;
    }

    static ceres::CostFunction* Create(const Vec2d& target_mean, const Vec2d& source_mean, const double& weight)
    {
        return new ceres::AutoDiffCostFunction<P2PMulKeyFrameCost, 2, 3>(new P2PMulKeyFrameCost(target_mean, source_mean, weight));
    }

private:
    const Vec2d target_mean_;
    const Vec2d source_mean_;
    const double weight_;
}; // class P2PCost


class P2LCost : public RegistrationCost{
public:

  P2LCost (const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean) :
    tar_mean_(target_mean),
    tar_normal_(target_normal),
    src_mean_(src_mean){}

  template <typename T>
  bool operator()(const T*  Ta,
                  const T*  Tb,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T,2,1> trans_mat_tar = GetTransMatrix2D(Ta);
    const Eigen::Matrix<T,2,2> rot_mat_tar = GetRotMatrix2D(Ta);

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_tar = (rot_mat_tar * (tar_mean_).cast<T>()) + trans_mat_tar;
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;

    const Eigen::Matrix<T,2,1> transformed_normal_tar = (rot_mat_tar * (tar_normal_).cast<T>());

    //T cost = T(0.0);
    const Eigen::Matrix<T,2,1> v = transformed_mean_src - transformed_mean_tar;

    const Eigen::Matrix<T,2,1> n = transformed_normal_tar;

    residuals_ptr[0] = v.dot(n);
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2LCost ,1, 3, 3>(new P2LCost (target_mean, target_normal, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_mean_;
  const Eigen::Vector2d tar_normal_;
  const Eigen::Vector2d src_mean_;
};
#endif // _RADAR_LOCALIZATION_CERES_REGISTRATION