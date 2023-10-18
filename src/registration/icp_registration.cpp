#include "icp_registration.h"

namespace registration{

void ICPRegistration::set_target(CLOUD::Ptr target_scan_)
{
    target_scan = target_scan_;
    build_target_kdtree();
}

void ICPRegistration::build_target_kdtree()
{
    kdtree.setInputCloud(target_scan);
}

bool ICPRegistration::registration(SE2 &init_pose, int iterations_)
{
    int iterations = iterations_;
    SE2 cur_pose = init_pose;
    double cost = 0, last_cost = 0;
    const float max_dis2 = 0.01;
    const int min_effect_pts = 20;

    for(int iter = 0; iter < iterations; ++iter){
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;

        for(size_t i = 0; i < source_scan -> points.size(); ++i){
            POINT p = cur_pose * source_scan -> points[i];
            std::vector<int> nn_idx;
            std::vector<float> nn_dis;
            kdtree.nearestKSearch(p, 1, nn_idx, nn_dis);

            if(nn_idx.size() > 0 && nn_dis[0] < max_dis2){
                effective_num++;

                Eigen::Matrix<double, 3, 2> J;
                double ex = -source_scan -> points[i].x * sin(cur_pose.get_theta()) +
                    source_scan -> points[i].y * cos(cur_pose.get_theta());
                double ey = -source_scan -> points[i].x * cos(cur_pose.get_theta()) -
                    source_scan -> points[i].y * sin(cur_pose.get_theta());
                J << 1, 0,
                     0, 1,
                     ex, ey;
                H += J * J.transpose();
                Eigen::Matrix<double, 2, 1> e(p.x - target_scan -> points[nn_idx[0]].x,
                    p.y - target_scan -> points[nn_idx[0]].y);
                b += -J * e;

                cost += e.dot(e);
            }
        }

        if(effective_num < min_effect_pts) return false;

        Vec3d dx = H.ldlt().solve(b);
        if(std::isnan(dx[0])) break;

        // 收敛条件
        cost /= effective_num;
        if(iter > 0 && cost >= last_cost) break;

        last_cost = cost;

        // 更新位姿
        cur_pose = SE2(dx[0], dx[1], dx[2]) * cur_pose;
    }
    init_pose = cur_pose;
    return true;
}


} // namespace registration