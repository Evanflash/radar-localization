#include "ceres_registration.hpp"

#include <unordered_set>

#include <opencv2/opencv.hpp>

#include <ceres/problem.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>

#include "normal_feature.hpp"
#include "radar_utils.hpp"

vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources, vector<double> init_pose, double thres)
{
    bool use_thres = true;
    MapFeatures targets_map = MapFeatures(targets, 1, 2);
    MapFeatures sources_map = MapFeatures(sources, 1, 2);

    // vector<double> parameters = vector<double>{0, 0, 0};
    vector<double> parameters = init_pose;
    vector<double> target_para = vector<double>{0, 0, 0};

    double pre_score = 0;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 构建问题
        ceres::Problem *problem = new ceres::Problem();
        problem -> AddParameterBlock(parameters.data(), 3);

        double angle_outlier = std::cos(M_PI / 6.0);

        double cur_radius;
        if(use_thres)
            cur_radius = thres;
        else
            cur_radius = (iterator == 1) ? 4 : 2;

        int nums = 0;
        // std::cout << sources_map.GetSize() << std::endl;

        CloudTypePtr visual_cloud(new CloudType());

        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            double sin_yaw = sin(parameters[2]);
            double cos_yaw = cos(parameters[2]);

            Eigen::Matrix3d T;
            T << cos_yaw, sin_yaw, parameters[0],
                -sin_yaw, cos_yaw, parameters[1],
                0, 0, 1;

            GridFeatures s = sources_map.GetGrid(i);

            Vec3d source_mean = T * Vec3d(s.u_(0), s.u_(1), 1);
            
            Vec2d source_mean_world = Vec2d(source_mean(0), source_mean(1));

            vector<GridFeatures> ts = targets_map.GetClosest(source_mean_world, cur_radius);
            
            
            PointType p(s.u_(0), s.u_(1), s.snormal_(0), s.snormal_(1));
            visual_cloud -> push_back(p);


            for(auto g : ts)
            {
                Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                Vec3d t_normal = Vec3d(g.snormal_(0), g.snormal_(1), 1);
                Vec3d src_normal_trans = T * s_normal;
                Vec3d tar_normal_trans = t_normal;
                double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                // double simi = 1;

                if(simi > angle_outlier)
                {
                    ceres::CostFunction *cost_function = P2PCost::Create(g.u_, s.u_);
                    problem -> AddResidualBlock(
                        cost_function, new ceres::HuberLoss(0.1), target_para.data(), parameters.data());
                        nums++;
                }
            }
        }

        CloudTypePtr visual_cloud_target(new CloudType());
        for(uint i = 0; i < targets_map.GetSize(); ++i)
        {
            GridFeatures g = targets_map.GetGrid(i);
            PointType p = PointType(g.u_(0), g.u_(1), g.snormal_(0), g.snormal_(1));
            visual_cloud_target -> push_back(p);
        }

        // cv::Mat image1 = pointcloud_to_cartesian_points(targets, 500, 500, 0.25);
        // cv::Mat image2 = pointcloud_to_cartesian_points(sources, 500, 500, 0.25);
        // cv::Mat image(500, 1000, CV_8U);
        // image1.copyTo(image.colRange(0, 500));
        // image2.copyTo(image.colRange(500, 1000));
        // cv::imshow("", image);
        // cv::waitKey(0);

        // std::cout << nums << std::endl;
        // solve
        problem -> SetParameterBlockConstant(target_para.data());

        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        // std::cout << summary_.BriefReport() << std::endl;


        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce) break;
        }
        pre_score = cur_sorce;
    }
    return parameters;
}

std::vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources, std::vector<double> init_pose, double thres, double rsize, double rsearch)
{
    bool use_thres = false;
    MapFeatures targets_map = MapFeatures(targets, rsize, rsearch);
    MapFeatures sources_map = MapFeatures(sources, rsize, rsearch);

    // vector<double> parameters = vector<double>{0, 0, 0};
    vector<double> parameters = init_pose;
    vector<double> target_para = vector<double>{0, 0, 0};

    double pre_score = 0;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 构建问题
        ceres::Problem *problem = new ceres::Problem();
        problem -> AddParameterBlock(parameters.data(), 3);

        double angle_outlier = std::cos(M_PI / 6.0);

        double cur_radius;
        if(use_thres)
            cur_radius = thres;
        else
            cur_radius = (iterator == 1) ? 4 : 2;

        int nums = 0;
        // std::cout << sources_map.GetSize() << std::endl;

        CloudTypePtr visual_cloud(new CloudType());

        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            double sin_yaw = sin(parameters[2]);
            double cos_yaw = cos(parameters[2]);

            Eigen::Matrix3d T;
            T << cos_yaw, sin_yaw, parameters[0],
                -sin_yaw, cos_yaw, parameters[1],
                0, 0, 1;

            GridFeatures s = sources_map.GetGrid(i);

            Vec3d source_mean = T * Vec3d(s.u_(0), s.u_(1), 1);
            
            Vec2d source_mean_world = Vec2d(source_mean(0), source_mean(1));

            vector<GridFeatures> ts = targets_map.GetClosest(source_mean_world, cur_radius);
            
            
            PointType p(s.u_(0), s.u_(1), s.snormal_(0), s.snormal_(1));
            visual_cloud -> push_back(p);


            for(auto g : ts)
            {
                Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                Vec3d t_normal = Vec3d(g.snormal_(0), g.snormal_(1), 1);
                Vec3d src_normal_trans = T * s_normal;
                Vec3d tar_normal_trans = t_normal;
                double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                // double simi = 1;

                if(simi > angle_outlier)
                {
                    ceres::CostFunction *cost_function = P2PCost::Create(g.u_, s.u_);
                    problem -> AddResidualBlock(
                        cost_function, new ceres::HuberLoss(0.1), target_para.data(), parameters.data());
                        nums++;
                }
            }
        }

        CloudTypePtr visual_cloud_target(new CloudType());
        for(uint i = 0; i < targets_map.GetSize(); ++i)
        {
            GridFeatures g = targets_map.GetGrid(i);
            PointType p = PointType(g.u_(0), g.u_(1), g.snormal_(0), g.snormal_(1));
            visual_cloud_target -> push_back(p);
        }

        cv::Mat image1 = pointcloud_to_arrow_image(visual_cloud, 800, 800, 0.2);
        cv::Mat image2 = pointcloud_to_arrow_image(visual_cloud_target, 800, 800, 0.2);
        cv::Mat image(800, 1600, CV_8U);
        image1.copyTo(image.colRange(0, 800));
        image2.copyTo(image.colRange(800, 1600));
        // cv::imshow("", image);
        // cv::waitKey(0);

        // std::cout << nums << std::endl;
        // solve
        problem -> SetParameterBlockConstant(target_para.data());

        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        // std::cout << summary_.BriefReport() << std::endl;


        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce) break;
        }
        pre_score = cur_sorce;
    }
    return parameters;
}

std::vector<double> P2PMulKeyFrameRegisterTest(vector<MapFeatures> clouds_, vector<vector<double>> transforms_, double thres)
{
    bool use_thres = true;
    // 取出目标点云与初始化位姿变换矩阵
    MapFeatures sources_map = clouds_.back();
    clouds_.pop_back();
    vector<double> parameters = transforms_.back();
    transforms_.pop_back();
    auto ParaToTransform = [](vector<double> para)
    {
        double cos_t = cos(para[2]);
        double sin_t = sin(para[2]);
        Eigen::Matrix3d T_;
        T_ << cos_t, sin_t, para[0],
             -sin_t, cos_t, para[1],
                            0, 0, 1;
        return T_;
    };
    // 初始配准得分
    double pre_score = 0;
    // 开始迭代优化
    vector<double> pre_parameters = parameters;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 初始化source到target的位姿变换矩阵
        std::unordered_set<uint> target_indexs;
        vector<Eigen::Matrix3d> T_stot_;
        for(uint i = 0; i < transforms_.size(); ++i)
        {
            T_stot_.push_back(ParaToTransform(transforms_[i]).inverse() * ParaToTransform(parameters));
        }
        // 构建问题
        ceres::Problem *problem = new ceres::Problem();
        problem -> AddParameterBlock(parameters.data(), 3);
        // 阈值
        double angle_outlier = std::cos(M_PI / 6.0);
        double cur_radius;
        if(use_thres)
            cur_radius = thres;
        else
            cur_radius = 2;//(iterator == 1) ? 4 : 2;

        int nums = 0;
        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            GridFeatures s = sources_map.GetGrid(i);
            // 模板点云参数
            vector<uint> indexs;
            vector<GridFeatures> grids;
            vector<double> weights;
            for(uint j = 0; j < clouds_.size(); ++j)
            {
                Vec3d source_mean = T_stot_[j] * Vec3d(s.u_(0), s.u_(1), 1);
                Vec2d source_mean_world = Vec2d(source_mean(0), source_mean(1));
                vector<GridFeatures> ts = clouds_[j].GetClosest(source_mean_world, cur_radius);
                for(auto g : ts)
                {
                    Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                    Vec3d t_normal = Vec3d(g.snormal_(0), g.snormal_(1), 1);
                    Vec3d src_normal_trans = T_stot_[j] * s_normal;
                    Vec3d tar_normal_trans = t_normal;
                    double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                    
                    if(simi > angle_outlier)
                    {
                        target_indexs.insert(j);
                        indexs.push_back(j);
                        grids.push_back(g);
                        double plan = 0;//2 * std::min(g.GetPlanarity(), s.GetPlanarity()) / (g.GetPlanarity() + s.GetPlanarity());
                        double det = 0;//2 * std::min(g.GetNsamples(), s.GetNsamples()) / (g.GetNsamples(), s.GetNsamples());
                        weights.push_back(plan + simi + det);
                        break;
                    }
                }
            }
            
            for(uint j = 0; j < indexs.size(); ++j)
            {
                ceres::LossFunction* cere_loss = 
                    new ceres::ScaledLoss(new ceres::HuberLoss(0.1), 1 /*+ weights[j]*/, ceres::TAKE_OWNERSHIP);
                // ceres::CostFunction *cost_function = P2LCost::Create(grids[j].u_, grids[j].snormal_, s.u_);
                ceres::CostFunction *cost_function = P2PCost::Create(grids[j].u_, s.u_);
                problem -> AddResidualBlock(
                    cost_function, cere_loss, transforms_[indexs[j]].data(), parameters.data());
                nums++;
            }
            // if(!indexs.empty()) nums++;     
        }
        for(auto i : target_indexs)
        {
            problem -> SetParameterBlockConstant(transforms_[i].data());
        }

        // solve
        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        // std::cout << nums << std::endl;
        // std::cout << summary_.BriefReport() << std::endl;


        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce)
            {
                parameters = pre_parameters;
                break;
            }
        }
        pre_score = cur_sorce;
        pre_parameters = parameters;
        delete problem;
    }
    return parameters;
}

std::vector<double> P2PMulKeyFrameRegisterInit(vector<MapFeatures> clouds_, vector<vector<double>> transforms_, double thres)
{
    bool use_thres = true;
    // 取出目标点云与初始化位姿变换矩阵
    MapFeatures sources_map = clouds_.back();
    clouds_.pop_back();
    vector<double> parameters = transforms_.back();
    transforms_.pop_back();
    auto ParaToTransform = [](vector<double> para)
    {
        double cos_t = cos(para[2]);
        double sin_t = sin(para[2]);
        Eigen::Matrix3d T_;
        T_ << cos_t, sin_t, para[0],
             -sin_t, cos_t, para[1],
                            0, 0, 1;
        return T_;
    };
    // 初始配准得分
    double pre_score = 0;
    // 开始迭代优化
    vector<double> pre_parameters = parameters;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 初始化source到target的位姿变换矩阵
        std::unordered_set<uint> target_indexs;
        vector<Eigen::Matrix3d> T_stot_;
        for(uint i = 0; i < transforms_.size(); ++i)
        {
            T_stot_.push_back(ParaToTransform(transforms_[i]).inverse() * ParaToTransform(parameters));
        }
        // 构建问题
        ceres::Problem *problem = new ceres::Problem();
        problem -> AddParameterBlock(parameters.data(), 3);
        // 阈值
        double angle_outlier = std::cos(M_PI / 6.0);
        double cur_radius;
        if(use_thres)
            cur_radius = thres;
        else
            cur_radius = (iterator == 1) ? 4 : 2;

        int nums = 0;
        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            GridFeatures s = sources_map.GetGrid(i);
            // 模板点云参数
            vector<uint> indexs;
            vector<GridFeatures> grids;
            vector<double> weights;
            for(uint j = 0; j < clouds_.size(); ++j)
            {
                Vec3d source_mean = T_stot_[j] * Vec3d(s.u_(0), s.u_(1), 1);
                Vec2d source_mean_world = Vec2d(source_mean(0), source_mean(1));
                vector<GridFeatures> ts = clouds_[j].GetClosest(source_mean_world, cur_radius);
                for(auto &g : ts)
                {
                    Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                    Vec3d t_normal = Vec3d(g.snormal_(0), g.snormal_(1), 1);
                    Vec3d src_normal_trans = T_stot_[j] * s_normal;
                    Vec3d tar_normal_trans = t_normal;
                    double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                    
                    if(simi > angle_outlier)
                    {
                        target_indexs.insert(j);
                        indexs.push_back(j);
                        grids.push_back(g);
                        double plan = 2 * std::min(g.GetPlanarity(), s.GetPlanarity()) / (g.GetPlanarity() + s.GetPlanarity());
                        double det = 0;//2 * std::min(g.GetNsamples(), s.GetNsamples()) / (g.GetNsamples(), s.GetNsamples());
                        weights.push_back(plan + simi + det);
                        break;
                    }
                }
            }
            
            for(uint j = 0; j < indexs.size(); ++j)
            {
                ceres::LossFunction* cere_loss = 
                    new ceres::ScaledLoss(new ceres::HuberLoss(0.1), /*weights[j] + */1, ceres::TAKE_OWNERSHIP);
                ceres::CostFunction *cost_function = P2LCost::Create(grids[j].u_, grids[j].snormal_, s.u_);
                // ceres::CostFunction *cost_function = P2PCost::Create(grids[j].u_, s.u_);
                problem -> AddResidualBlock(
                    cost_function, cere_loss, transforms_[indexs[j]].data(), parameters.data());
                nums++;
            }
            // if(!indexs.empty()) nums++;     
        }
        for(auto i : target_indexs)
        {
            problem -> SetParameterBlockConstant(transforms_[i].data());
        }

        // solve
        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        // std::cout << nums << std::endl;
        // std::cout << summary_.BriefReport() << std::endl;


        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce)
            {
                parameters = pre_parameters;
                break;
            }
        }
        pre_score = cur_sorce;
        pre_parameters = parameters;
        delete problem;
    }
    return parameters;
}

vector<double> P2PRegister(MapFeatures targets, MapFeatures sources, vector<double> init_pose)
{
    MapFeatures targets_map = targets;
    MapFeatures sources_map = sources;

    vector<double> parameters = init_pose;
    vector<double> target_para = vector<double>{0, 0, 0};

    double pre_score = 0;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 构建问题
        ceres::Problem *problem = new ceres::Problem();
        problem -> AddParameterBlock(parameters.data(), 3);
        bool have = false;
        double angle_outlier = std::cos(M_PI / 6.0);

        double cur_radius = (iterator == 1) ? 4 : 2;;

        int nums = 0;

        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            double sin_yaw = sin(parameters[2]);
            double cos_yaw = cos(parameters[2]);

            Eigen::Matrix3d T;
            T << cos_yaw, sin_yaw, parameters[0],
                -sin_yaw, cos_yaw, parameters[1],
                0, 0, 1;

            GridFeatures s = sources_map.GetGrid(i);

            Vec3d source_mean = T * Vec3d(s.u_(0), s.u_(1), 1);
            
            Vec2d source_mean_world = Vec2d(source_mean(0), source_mean(1));

            vector<GridFeatures> ts = targets_map.GetClosest(source_mean_world, cur_radius);

            for(auto g : ts)
            {
                Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                Vec3d t_normal = Vec3d(g.snormal_(0), g.snormal_(1), 1);
                Vec3d src_normal_trans = T * s_normal;
                Vec3d tar_normal_trans = t_normal;
                double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);

                if(simi > angle_outlier)
                {
                    // ceres::CostFunction *cost_function = P2PCost::Create(g.u_, s.u_);
                    ceres::CostFunction *cost_function = P2LCost::Create(g.u_, g.snormal_, s.u_);
                    problem -> AddResidualBlock(
                        cost_function, new ceres::HuberLoss(0.1), target_para.data(), parameters.data());
                        nums++;
                    have = true;
                }
            }
        }

        // solve
        if(have)    problem -> SetParameterBlockConstant(target_para.data());

        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
 
        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce) break;
        }
        pre_score = cur_sorce;
        delete problem;
    }
    return parameters;
}