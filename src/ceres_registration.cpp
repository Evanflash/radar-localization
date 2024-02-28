#include "ceres_registration.hpp"

#include <ceres/problem.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>

#include "normal_feature.hpp"

vector<double> P2PRegisterTest(CloudTypePtr targets, CloudTypePtr sources)
{
    MapFeatures targets_map = MapFeatures(targets, 2, 3);
    MapFeatures sources_map = MapFeatures(sources, 2, 3);

    vector<double> parameters = vector<double>{0, 0, 0};
    vector<double> target_para = vector<double>{0, 0, 0};

    double pre_score = 0;
    for(int iterator = 1; iterator <= 10; ++iterator)
    {
        // 构建问题
        ceres::Problem *problem;
        problem -> AddParameterBlock(parameters.data(), 3);

        double angle_outlier = std::cos(M_PI / 6.0);

        double cur_radius = (iterator == 1) ? 4 : 2;

        for(uint i = 0; i < sources_map.GetSize(); ++i)
        {
            GridFeatures s = sources_map.GetGrid(i);

            vector<GridFeatures*> ts = targets_map.GetClosest(s.u_, cur_radius);

            double sin_yaw = sin(parameters[2]);
            double cos_yaw = cos(parameters[2]);
            Eigen::Matrix3d T;
            T << cos_yaw, -sin_yaw, parameters[0],
                sin_yaw, cos_yaw, parameters[1],
                0, 0, 1;

            for(auto g : ts)
            {
                // Vec2d src_normal_trans = T * s.snormal_;
                // Vec2d tar_normal_trans = g -> snormal_;
                // double simi = std::max(src_normal_trans.dot(tar_normal_trans), 0.0);
                double simi = 1;

                if(simi > angle_outlier)
                {
                    ceres::CostFunction *cost_function = P2PCost::Create(s.u_, g -> u_);
                    problem -> AddResidualBlock(
                        cost_function, new ceres::HuberLoss(0.1), target_para.data(), parameters.data());
                }
            }
        }
        // solve
        ceres::Solver::Options options_;
        ceres::Solver::Summary summary_;
        ceres::Solve(options_, problem, &summary_);
        std::cout << summary_.BriefReport() << std::endl;

        double cur_sorce = summary_.final_cost;

        if(iterator > 3)
        {
            if(pre_score < cur_sorce) break;
        }
        pre_score = cur_sorce;
    }
    return parameters;
}