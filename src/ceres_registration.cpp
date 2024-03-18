#include "ceres_registration.hpp"

#include <opencv2/opencv.hpp>

#include <ceres/problem.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>

#include "normal_feature.hpp"
#include "radar_utils.hpp"

vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources, vector<double> init_pose)
{
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

        double cur_radius = (iterator == 1) ? 4 : 2;

        int nums = 0;
        std::cout << sources_map.GetSize() << std::endl;

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

            vector<GridFeatures*> ts = targets_map.GetClosest(source_mean_world, cur_radius);
            
            
            PointType p(s.u_(0), s.u_(1), s.snormal_(0), s.snormal_(1));
            visual_cloud -> push_back(p);


            for(auto g : ts)
            {
                Vec3d s_normal = Vec3d(s.snormal_(0), s.snormal_(1), 1);
                Vec3d t_normal = Vec3d(g -> snormal_(0), g -> snormal_(1), 1);
                Vec3d src_normal_trans = T * s_normal;
                Vec3d tar_normal_trans = t_normal;
                double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                // double simi = 1;

                if(simi > angle_outlier)
                {
                    ceres::CostFunction *cost_function = P2PCost::Create(g -> u_, s.u_);
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

        std::cout << nums << std::endl;
        // solve
        problem -> SetParameterBlockConstant(target_para.data());

        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        std::cout << summary_.BriefReport() << std::endl;


        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score <= cur_sorce) break;
        }
        pre_score = cur_sorce;
    }
    return parameters;
}