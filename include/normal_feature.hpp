#ifndef _RADAR_LOCALIZATION_NORMAL_FEATURE
#define _RADAR_LOCALIZATION_NORMAL_FEATURE

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

using std::vector;
using Vec2d = Eigen::Vector2d;
using Mat2d = Eigen::Matrix2d;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<pcl::PointXYZI>;
using CloudTypePtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

class GridFeatures
{
public:
    GridFeatures(CloudTypePtr cloud, const vector<int> &inds, const Vec2d &origin);

    bool ComputeNormal(const Vec2d & origin);

    double GetPlanarity() const;
    double GetNsamples() const;

public:
    Vec2d u_ = Vec2d::Zero();
    Mat2d cov_ = Mat2d::Identity() * 0.1;
    Vec2d snormal_, orth_normal_;
    double lambda_min, lambda_max;
    size_t Nsamples_;
    // 判断是否有效
    bool vaild;
}; // class GridFeatures

class MapFeatures
{  
public:
    MapFeatures(CloudTypePtr cloud, float grid_size, float radius);

    vector<GridFeatures*> GetClosest(Vec2d &p, double d);

    size_t GetSize() const;

    GridFeatures GetGrid(size_t i) const;

private:
    void ComputeNormals(const Vec2d &origin);
    void ComputeSearchTreeFormGrids();

private:
    // 特征点
    vector<GridFeatures> grids;
    // 搜索树
    pcl::KdTreeFLANN<pcl::PointXY> grid_search_tree;
    // 参数
    CloudTypePtr cloud_;  // 输入点云
    float grid_size_;     // 网格大小
    float radius_;        // 搜索范围

}; // class MapFeatures

#endif // _RADAR_LOCALIZATION_NORMAL_FEATURE