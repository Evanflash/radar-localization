#include "normal_feature.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

GridFeatures::GridFeatures(CloudTypePtr cloud, const vector<int> &inds, const Vec2d &origin)
    : Nsamples_(inds.size())
{
    Eigen::MatrixXd x(Nsamples_, 2);
    for(uint i = 0; i < Nsamples_; ++i)
    {
        x.block<1, 2>(i, 0) = Vec2d(cloud -> points[inds[i]].x, cloud -> points[inds[i]].y);
    }

    // mean
    for(uint i = 0; i < Nsamples_; ++i)
    {
        u_.block<2, 1>(0, 0) += x.block<1, 2>(i, 0).transpose() / Nsamples_;
    }

    // 减去均值
    for(uint i = 0; i < Nsamples_; ++i)
    {
        x.block<1, 2>(i, 0) = x.block<1, 2>(i, 0) - u_.block<2, 1>(0, 0).transpose();
    }

    cov_ = x.transpose() * x;
    vaild = ComputeNormal(origin);
}

bool GridFeatures::ComputeNormal(const Vec2d & origin)
{
    Eigen::SelfAdjointEigenSolver<Mat2d> es(cov_);
    es.compute(cov_);

    snormal_ = es.eigenvectors().col(0);
    orth_normal_ = es.eigenvectors().col(1);
    lambda_min = es.eigenvalues()[0];
    lambda_max =  es.eigenvalues()[1];

    const bool cov_reasonalbe = (fabs(lambda_max / lambda_min) <= 10000) &&
        (lambda_max * lambda_min > 0.00001) && 
        lambda_min > 0 &&
        lambda_max > 0;
    
    Vec2d po_u = origin - u_;
    if(snormal_.dot(po_u) < 0)
        snormal_ = - snormal_;
    
    return cov_reasonalbe;
}

double GridFeatures::GetPlanarity() const
{
    return log10(1 + abs(lambda_max / lambda_min));
}
double GridFeatures::GetNsamples() const
{
    return (double)Nsamples_;
}

MapFeatures::MapFeatures(CloudTypePtr cloud, float grid_size, float radius)
{
    cloud_ = cloud;
    grid_size_ = grid_size;
    radius_ = radius;

    ComputeNormals(Vec2d(0, 0));
    ComputeSearchTreeFormGrids();
}

void MapFeatures::ComputeNormals(const Vec2d &origin)
{
    pcl::search::KdTree<PointType> kdtree_input;
    CloudType grid_sample_means;

    kdtree_input.setInputCloud(cloud_);
    // for(uint i = 0; i < cloud_ ->size(); ++i)
    // {
    //     PointType p = cloud_ -> points[i];
    //     if(std::isnan(p.x))
    //     {
    //         std::cout << p << std::endl;
    //     }
        
    // }
    // 计算均值
    pcl::VoxelGrid<PointType> downsampleFilter;
    downsampleFilter.setInputCloud(cloud_);
    downsampleFilter.setLeafSize(grid_size_, grid_size_, grid_size_);
    downsampleFilter.filter(grid_sample_means);
    // removeNaNFromPointCloud(grid_sample_means);
    // 查询特征点是否满足周围点数大于阈值的条件
    int num_thres = 12;

    vector<int> point_ind;
    vector<float> point_dis;
    for(uint i = 0; i < grid_sample_means.size(); ++i)
    {
        point_ind.clear();
        point_dis.clear();
        PointType pnt = grid_sample_means.points[i];
        // std::cout << pnt << std::endl;
        if(kdtree_input.radiusSearchT(pnt, radius_, point_ind, point_dis) >= num_thres)
        {
            GridFeatures grid = GridFeatures(cloud_, point_ind, origin);
            if(grid.vaild)
                grids.push_back(grid);
        }
    }
}

void MapFeatures::ComputeSearchTreeFormGrids()
{
    pcl::PointCloud<pcl::PointXY>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXY>());
    for(auto g : grids)
    {
        pcl::PointXY p;
        p.x = g.u_(0);
        p.y = g.u_(1);
        feature_cloud -> push_back(p);
    }
    grid_search_tree.setSortedResults(true);
    grid_search_tree.setInputCloud(feature_cloud);
}

vector<GridFeatures*> MapFeatures::GetClosest(Vec2d &p, double d)
{
    vector<GridFeatures*> nearby;
    
    pcl::PointXY pnt;
    pnt.x = p[0];
    pnt.y = p[1];

    vector<int> point_ind;
    vector<float> point_dis;
    grid_search_tree.nearestKSearch(pnt, 1, point_ind, point_dis);
    if(!point_ind.empty() && point_dis[0] < d * d){
        nearby.push_back(&grids[point_ind[0]]);
    }
    return nearby;
}

size_t MapFeatures::GetSize() const
{
    return grids.size();
}

GridFeatures MapFeatures::GetGrid(size_t i) const
{
    return grids[i];
}
