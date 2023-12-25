#include "registration.hpp"
#include "filter.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>

Eigen::Matrix4f pcl_icp_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, int iterators)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterators);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    CloudT::Ptr output_cloud(new CloudT());
    icp.align(*output_cloud);
    
    return icp.getFinalTransformation();
}

Eigen::Vector3f point_to_line_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3f init_pose)
{
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_points(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_surf_points -> setInputCloud(target_cloud);
    int neighbor_num = 5;

    int iterations = 30;
    float cost = 0, last_cost = 0;

    Eigen::Vector3f result_pose = init_pose;

    // 位姿变换
    auto transpose = [&](const pcl::PointXYZI &point)
    {
        float cos_theta = cos(result_pose[2]);
        float sin_theta = sin(result_pose[2]);
        Eigen::Matrix3f T;
        T << cos_theta, sin_theta, result_pose[0],
             -sin_theta,  cos_theta, result_pose[1],
             0, 0, 1;
        Eigen::Vector3f point_trans(point.x, point.y, 1);
        point_trans = T * point_trans;
        return pcl::PointXYZI(point_trans[0], point_trans[1], 0, point.intensity);
    };

    // 高斯迭代
    for(int iterCount = 0; iterCount < iterations; iterCount++){
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f B = Eigen::Vector3f::Zero();

        cost = 0;    
        int nums = 0;
        for(int i = 0; i < (int)source_cloud -> size(); ++i){
            pcl::PointXYZI point_ori = source_cloud -> points[i];
            pcl::PointXYZI point_trans = transpose(point_ori);

            std::vector<int> nn_idx(neighbor_num);
            std::vector<float> nn_distance(neighbor_num);

            kdtree_surf_points -> nearestKSearch(point_trans, neighbor_num, nn_idx, nn_distance);
            if(nn_distance.back() < 5.0){
                float cx = 0, cy = 0;
                for(int j = 0; j < neighbor_num; ++j){
                    cx += target_cloud -> points[nn_idx[j]].x;
                    cy += target_cloud -> points[nn_idx[j]].y;
                }
                cx /= neighbor_num;
                cy /= neighbor_num;
                
                float a11 = 0, a12 = 0, a21 = 0, a22 = 0;
                for(int j = 0; j < neighbor_num; ++j){
                    float ax = target_cloud -> points[nn_idx[j]].x - cx;
                    float ay = target_cloud -> points[nn_idx[j]].y - cy;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a21 += ay * ax;
                    a22 += ay * ay;
                }
                a11 /= neighbor_num;
                a12 /= neighbor_num;
                a21 /= neighbor_num;
                a22 /= neighbor_num;

                // 计算特征值
                Eigen::Matrix2f matA = Eigen::Matrix2f::Zero();
                matA(0, 0) = a11;
                matA(0, 1) = a12;
                matA(1, 0) = a21;
                matA(1, 1) = a22;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> esolver(matA);

                Eigen::Matrix<float, 1, 2> matD = esolver.eigenvalues();
                Eigen::Matrix2f matV = esolver.eigenvectors();

                if(matD[1] > 2 * matD[0]){
                    float x0 = point_trans.x;
                    float y0 = point_trans.y;
                    float x1 = cx + 0.5 * matV(0, 1);
                    float y1 = cy + 0.5 * matV(1, 1);
                    float x2 = cx - 0.5 * matV(0, 1);
                    float y2 = cy - 0.5 * matV(1, 1);

                    // float x0 = point_trans.x;
                    // float y0 = point_trans.y;
                    // float x1 = target_cloud -> points[nn_idx[0]].x;
                    // float y1 = target_cloud -> points[nn_idx[0]].y;
                    // float x2 = target_cloud -> points[nn_idx[1]].x;
                    // float y2 = target_cloud -> points[nn_idx[1]].y;
                    
                    float a = sqrt((x0 - x1) * (x0 - x1) +
                                    (y0 - y1) * (y0 - y1));
                    float b = sqrt((x0 - x2) * (x0 - x2) +
                                    (y0 - y2) * (y0 - y2));
                    float c = sqrt((x1 - x2) * (x1 - x2) +
                                    (y1 - y2) * (y1 - y2));
                    
                    // 海伦公式计算面积
                    float l = (a + b + c) / 2;
                    float S = sqrt(l * (l - a) * (l - b) * (l - c));

                    float error = S;
                    // std::cout << "error = " << error << std::endl;
                    // std::cout << "x0 = " << x0 << ", y0 = " << y0 <<
                    //     ", x1 = " << x1 << ", y1 = " << y1 <<
                    //     ", x2 = " << x2 << ", y2 = " << y2 << std::endl;

                    // 计算雅克比矩阵
                    float theta = result_pose[2];
                    Eigen::Vector3f dx0(1, 0, 
                        -point_ori.x * sin(theta) + point_ori.y * cos(theta));
                    Eigen::Vector3f dy0(0, 1,
                        -point_ori.x * cos(theta) - point_ori.y * sin(theta));

                    Eigen::Vector3f da = (1 / a) * 
                        ((x0 - x1) * dx0 + (y0 - y1) * dy0);
                    Eigen::Vector3f db = (1 / b) *
                        ((x0 - x2) * dx0 + (y0 - y2) * dy0);

                    Eigen::Vector3f J = (1 / (8 * S)) * 
                        (-a * a * a * da - b * b * b * db + 
                        a * b * b * da + a * a * b * db + 
                        a * c * c * da + b * c * c * db);

                    // std::cout << "J = [" << J[0] << " " << J[1] <<
                    //     " " << J[2] << "]" << std::endl;
                    if(!finite(J[0])) continue;

                    H += J * J.transpose();
                    B += -error * J;
                    cost += error * error;

                    nums++;
                }
            }
            
        }
        
        // 求解 Hx = B
        Eigen::Vector3f dx = H.ldlt().solve(B);
        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }

        cost = cost / nums;
        if(iterCount > 0 && last_cost < cost) break;

        result_pose = result_pose + dx;
        last_cost = cost;

        std::cout << "------------------------------------------" << std::endl;
        std::cout << nums << std::endl;
        std::cout << "cost = " << cost / nums << std::endl;

        std::cout << "x = " << result_pose[0] << ", y = " << result_pose[1] << ", theta = " << result_pose[2] << std::endl;
    }

    return result_pose;
}

Eigen::Vector3f point_to_line_registration_weighted(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud, 
    Eigen::Vector3f init_pose)
{
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_points(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_surf_points -> setInputCloud(target_cloud);
    int neighbor_num = 6;

    int iterations = 30;
    float cost = 0, last_cost = 0;

    Eigen::Vector3f result_pose = init_pose;
    // std::cout << "begin" << std::endl;
    // 位姿变换
    auto transpose = [&](const pcl::PointXYZI &point)
    {
        float cos_theta = cos(result_pose[2]);
        float sin_theta = sin(result_pose[2]);
        Eigen::Matrix3f T;
        T << cos_theta, sin_theta, result_pose[0],
             -sin_theta,  cos_theta, result_pose[1],
             0, 0, 1;
        Eigen::Vector3f point_trans(point.x, point.y, 1);
        point_trans = T * point_trans;
        return pcl::PointXYZI(point_trans[0], point_trans[1], 0, point.intensity);
    };

    std::vector<std::vector<pcl::PointXYZI>> source_set = divide_into_grid(source_cloud, 2, 5);
    // 高斯迭代
    for(int iterCount = 0; iterCount < iterations; iterCount++){
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f B = Eigen::Vector3f::Zero();

        cost = 0;    
        int nums = 0;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
        // source_cloud_tmp -> reserve(source_cloud -> size());
        // for(auto point : source_cloud -> points){
        //     source_cloud_tmp -> push_back(transpose(point));
        // }

        for(auto one_set : source_set){
            for(auto &set_point : one_set){
                set_point = transpose(set_point);
            }

            Eigen::Vector2f source_mean;
            Eigen::Matrix<float, 1, 2> source_matD;
            Eigen::Matrix2f source_matV;
            calculate_mean_and_cov(one_set, source_mean, source_matD, source_matV);

            pcl::PointXYZI point_ori = pcl::PointXYZI(source_mean[0], source_mean[1], 0, 0);
            pcl::PointXYZI point_trans = point_ori;//transpose(point_ori);

            std::vector<int> nn_idx(neighbor_num);
            std::vector<float> nn_distance(neighbor_num);

            kdtree_surf_points -> nearestKSearch(point_trans, neighbor_num, nn_idx, nn_distance);
            if(nn_distance.back() < 5.0){
                std::vector<pcl::PointXYZI> target_points_set;
                for(auto ind : nn_idx){
                    target_points_set.push_back(target_cloud -> points[ind]);
                }
                Eigen::Vector2f target_mean;
                Eigen::Matrix<float, 1, 2> target_matD;
                Eigen::Matrix2f target_matV;
                calculate_mean_and_cov(target_points_set, target_mean, target_matD, target_matV);

                if(target_matD[1] > 2 * target_matD[0]){
                    float x0 = point_trans.x;
                    float y0 = point_trans.y;

                    float weight = sqrt(abs(target_matV(0, 0) * source_matV(0, 0) + 
                        target_matV(1, 0) * source_matV(1, 0)));

                    float x1 = target_mean[0] + weight * target_matV(0, 1);
                    float y1 = target_mean[1] + weight * target_matV(1, 1);
                    float x2 = target_mean[0] - weight * target_matV(0, 1);
                    float y2 = target_mean[1] - weight * target_matV(1, 1);
 
                    float a = sqrt((x0 - x1) * (x0 - x1) +
                                    (y0 - y1) * (y0 - y1));
                    float b = sqrt((x0 - x2) * (x0 - x2) +
                                    (y0 - y2) * (y0 - y2));
                    float c = sqrt((x1 - x2) * (x1 - x2) +
                                    (y1 - y2) * (y1 - y2));
                    
                    // 海伦公式计算面积
                    float l = (a + b + c) / 2;
                    float S = sqrt(l * (l - a) * (l - b) * (l - c));

                    float error = S;
                    // std::cout << "error = " << error << std::endl;
                    // std::cout << "x0 = " << x0 << ", y0 = " << y0 <<
                    //     ", x1 = " << x1 << ", y1 = " << y1 <<
                    //     ", x2 = " << x2 << ", y2 = " << y2 << std::endl;

                    // 计算雅克比矩阵
                    float theta = result_pose[2];
                    Eigen::Vector3f dx0(1, 0, 
                        -point_ori.x * sin(theta) + point_ori.y * cos(theta));
                    Eigen::Vector3f dy0(0, 1,
                        -point_ori.x * cos(theta) - point_ori.y * sin(theta));

                    Eigen::Vector3f da = (1 / a) * 
                        ((x0 - x1) * dx0 + (y0 - y1) * dy0);
                    Eigen::Vector3f db = (1 / b) *
                        ((x0 - x2) * dx0 + (y0 - y2) * dy0);

                    Eigen::Vector3f J = (1 / (8 * S)) * 
                        (-a * a * a * da - b * b * b * db + 
                        a * b * b * da + a * a * b * db + 
                        a * c * c * da + b * c * c * db);

                    // std::cout << "J = [" << J[0] << " " << J[1] <<
                    //     " " << J[2] << "]" << std::endl;
                    if(!finite(J[0])) continue;

                    H += J * J.transpose();
                    B += -error * J;
                    cost += error * error;

                    nums++;
                }
            }
            
        }
        
        // 求解 Hx = B
        Eigen::Vector3f dx = H.ldlt().solve(B);
        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }

        cost = cost / nums;
        if(iterCount > 0 && last_cost < cost) break;

        result_pose = result_pose + dx;
        last_cost = cost;

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << nums << std::endl;
        // std::cout << "cost = " << cost << std::endl;

        // std::cout << "x = " << result_pose[0] << ", y = " << result_pose[1] << ", theta = " << result_pose[2] << std::endl;
    }
    // std::cout << "end" << std::endl;
    return result_pose;
}

Eigen::Vector3f point_to_line_registration_weighted_mulkeyframe(
    CloudT::Ptr source_cloud, std::queue<CloudT::Ptr> target_clouds, Eigen::Vector3f init_pose)
{
    int neighbor_num = 6;

    int iterations = 30;
    float cost = 0, last_cost = 0;

    Eigen::Vector3f result_pose = init_pose;

    std::vector<CloudT::Ptr> target_clouds_vec;
    while(!target_clouds.empty()){
        target_clouds_vec.push_back(target_clouds.front());
        target_clouds.pop();
    }

    // std::cout << "begin" << std::endl;
    // 位姿变换
    auto transpose = [&](const pcl::PointXYZI &point)
    {
        float cos_theta = cos(result_pose[2]);
        float sin_theta = sin(result_pose[2]);
        Eigen::Matrix3f T;
        T << cos_theta, sin_theta, result_pose[0],
             -sin_theta,  cos_theta, result_pose[1],
             0, 0, 1;
        Eigen::Vector3f point_trans(point.x, point.y, 1);
        point_trans = T * point_trans;
        return pcl::PointXYZI(point_trans[0], point_trans[1], 0, point.intensity);
    };

    // 高斯迭代
    for(int iterCount = 0; iterCount < iterations; iterCount++){
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f B = Eigen::Vector3f::Zero();

        cost = 0;    
        int nums = 0;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
        // source_cloud_tmp -> reserve(source_cloud -> size());
        // for(auto point : source_cloud -> points){
        //     source_cloud_tmp -> push_back(transpose(point));
        // }

        std::vector<std::vector<pcl::PointXYZI>> source_set = divide_into_grid(source_cloud, 2, 5);
        for(auto &one_set : source_set){
            for(auto &set_point : one_set){
                set_point = transpose(set_point);
            }
        }
        
        for(auto target_cloud : target_clouds_vec){
            pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_points(new pcl::KdTreeFLANN<pcl::PointXYZI>());
            kdtree_surf_points -> setInputCloud(target_cloud);

            for(auto one_set : source_set){
                Eigen::Vector2f source_mean;
                Eigen::Matrix<float, 1, 2> source_matD;
                Eigen::Matrix2f source_matV;
                calculate_mean_and_cov(one_set, source_mean, source_matD, source_matV);

                pcl::PointXYZI point_ori = pcl::PointXYZI(source_mean[0], source_mean[1], 0, 0);
                pcl::PointXYZI point_trans = point_ori;//transpose(point_ori);

                std::vector<int> nn_idx(neighbor_num);
                std::vector<float> nn_distance(neighbor_num);

                kdtree_surf_points -> nearestKSearch(point_trans, neighbor_num, nn_idx, nn_distance);
                if(nn_distance.back() < 5.0){
                    std::vector<pcl::PointXYZI> target_points_set;
                    for(auto ind : nn_idx){
                        target_points_set.push_back(target_cloud -> points[ind]);
                    }
                    Eigen::Vector2f target_mean;
                    Eigen::Matrix<float, 1, 2> target_matD;
                    Eigen::Matrix2f target_matV;
                    calculate_mean_and_cov(target_points_set, target_mean, target_matD, target_matV);

                    if(target_matD[1] > 2 * target_matD[0]){
                        float x0 = point_trans.x;
                        float y0 = point_trans.y;

                        float weight = sqrt(abs(target_matV(0, 0) * source_matV(0, 0) + 
                            target_matV(1, 0) * source_matV(1, 0)));

                        if(weight < 0.95) continue;

                        float x1 = target_mean[0] + weight * target_matV(0, 1);
                        float y1 = target_mean[1] + weight * target_matV(1, 1);
                        float x2 = target_mean[0] - weight * target_matV(0, 1);
                        float y2 = target_mean[1] - weight * target_matV(1, 1);
    
                        float a = sqrt((x0 - x1) * (x0 - x1) +
                                        (y0 - y1) * (y0 - y1));
                        float b = sqrt((x0 - x2) * (x0 - x2) +
                                        (y0 - y2) * (y0 - y2));
                        float c = sqrt((x1 - x2) * (x1 - x2) +
                                        (y1 - y2) * (y1 - y2));
                        
                        // 海伦公式计算面积
                        float l = (a + b + c) / 2;
                        float S = sqrt(l * (l - a) * (l - b) * (l - c));

                        float error = S;
                        // std::cout << "error = " << error << std::endl;
                        // std::cout << "x0 = " << x0 << ", y0 = " << y0 <<
                        //     ", x1 = " << x1 << ", y1 = " << y1 <<
                        //     ", x2 = " << x2 << ", y2 = " << y2 << std::endl;

                        // 计算雅克比矩阵
                        float theta = result_pose[2];
                        Eigen::Vector3f dx0(1, 0, 
                            -point_ori.x * sin(theta) + point_ori.y * cos(theta));
                        Eigen::Vector3f dy0(0, 1,
                            -point_ori.x * cos(theta) - point_ori.y * sin(theta));

                        Eigen::Vector3f da = (1 / a) * 
                            ((x0 - x1) * dx0 + (y0 - y1) * dy0);
                        Eigen::Vector3f db = (1 / b) *
                            ((x0 - x2) * dx0 + (y0 - y2) * dy0);

                        Eigen::Vector3f J = (1 / (8 * S)) * 
                            (-a * a * a * da - b * b * b * db + 
                            a * b * b * da + a * a * b * db + 
                            a * c * c * da + b * c * c * db);

                        // std::cout << "J = [" << J[0] << " " << J[1] <<
                        //     " " << J[2] << "]" << std::endl;
                        if(!finite(J[0])) continue;

                        H += J * J.transpose();
                        B += -error * J;
                        cost += error * error;

                        nums++;
                    }
                }
                
            }
        }

        // 求解 Hx = B
        Eigen::Vector3f dx = H.ldlt().solve(B);
        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }

        cost = cost / nums;
        if(iterCount > 0 && last_cost < cost) break;

        result_pose = result_pose + dx;
        last_cost = cost;

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << nums << std::endl;
        // std::cout << "cost = " << cost << std::endl;

        // std::cout << "x = " << result_pose[0] << ", y = " << result_pose[1] << ", theta = " << result_pose[2] << std::endl;
    }
    // std::cout << "end" << std::endl;
    return result_pose;
}

Eigen::Vector3f common_P2L_registration(CloudT::Ptr source_cloud, CloudT::Ptr target_cloud,
    Eigen::Vector3f init_pose)
{
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_points(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_surf_points -> setInputCloud(target_cloud);
    int neighbor_num = 5;

    int iterations = 30;
    float cost = 0, last_cost = 0;

    Eigen::Vector3f result_pose = init_pose;

    // 位姿变换
    auto transpose = [&](const pcl::PointXYZI &point)
    {
        float cos_theta = cos(result_pose[2]);
        float sin_theta = sin(result_pose[2]);
        Eigen::Matrix3f T;
        T << cos_theta, sin_theta, result_pose[0],
             -sin_theta,  cos_theta, result_pose[1],
             0, 0, 1;
        Eigen::Vector3f point_trans(point.x, point.y, 1);
        point_trans = T * point_trans;
        return pcl::PointXYZI(point_trans[0], point_trans[1], 0, point.intensity);
    };

    // 高斯迭代
    for(int iterCount = 0; iterCount < iterations; iterCount++){
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f B = Eigen::Vector3f::Zero();

        cost = 0;    
        int nums = 0;
        for(int i = 0; i < (int)source_cloud -> size(); ++i){
            pcl::PointXYZI point_ori = source_cloud -> points[i];
            pcl::PointXYZI point_trans = transpose(point_ori);

            std::vector<int> nn_idx(neighbor_num);
            std::vector<float> nn_distance(neighbor_num);

            kdtree_surf_points -> nearestKSearch(point_trans, neighbor_num, nn_idx, nn_distance);
            if(nn_distance.back() < 5.0){
                float cx = 0, cy = 0;
                for(int j = 0; j < neighbor_num; ++j){
                    cx += target_cloud -> points[nn_idx[j]].x;
                    cy += target_cloud -> points[nn_idx[j]].y;
                }
                cx /= neighbor_num;
                cy /= neighbor_num;
                
                float a11 = 0, a12 = 0, a21 = 0, a22 = 0;
                for(int j = 0; j < neighbor_num; ++j){
                    float ax = target_cloud -> points[nn_idx[j]].x - cx;
                    float ay = target_cloud -> points[nn_idx[j]].y - cy;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a21 += ay * ax;
                    a22 += ay * ay;
                }
                a11 /= neighbor_num;
                a12 /= neighbor_num;
                a21 /= neighbor_num;
                a22 /= neighbor_num;

                // 计算特征值
                Eigen::Matrix2f matA = Eigen::Matrix2f::Zero();
                matA(0, 0) = a11;
                matA(0, 1) = a12;
                matA(1, 0) = a21;
                matA(1, 1) = a22;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> esolver(matA);

                Eigen::Matrix<float, 1, 2> matD = esolver.eigenvalues();
                Eigen::Matrix2f matV = esolver.eigenvectors();

                if(matD[1] > 2 * matD[0]){
                    float x0 = point_trans.x;
                    float y0 = point_trans.y;

                    float error = matV(0, 0) * (cx - x0) + matV(0, 1) * (cy - y0);


                    // 计算雅克比矩阵
                    float theta = result_pose[2];
                    Eigen::Matrix<float, 3, 2> tmp;
                    tmp << -1, 0, 
                        0, -1, 
                        (sin(theta) - cos(theta)), (cos(theta) + sin(theta));
                    Eigen::Vector3f J = tmp * Eigen::Vector2f(matV(0, 0), matV(0, 1));

                    if(!finite(J[0])) continue;

                    H += J * J.transpose();
                    B += -error * J;
                    cost += error * error;

                    nums++;
                }
            }
            
        }
        
        // 求解 Hx = B
        Eigen::Vector3f dx = H.ldlt().solve(B);
        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }
        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << nums << std::endl;
        // std::cout << "cost = " << cost << std::endl;

        // std::cout << "x = " << result_pose[0] << ", y = " << result_pose[1] << ", theta = " << result_pose[2] << std::endl;

        cost = cost / nums;
        if(iterCount > 0 && last_cost < cost) break;

        result_pose = result_pose + dx;
        last_cost = cost;
    }

    return result_pose;
}

float huber_robust_core(float cost, float thres)
{
    if(cost > thres)
        return (thres * (abs(cost) - 0.5 * thres));
    else    
        return cost * cost;
}

void calculate_mean_and_cov(std::vector<pcl::PointXYZI> point_set,
    Eigen::Vector2f& mean, Eigen::Matrix<float, 1, 2>& matD, Eigen::Matrix2f& matV)
{
    mean = Eigen::Vector2f::Zero();
    matD = Eigen::Matrix<float, 1, 2>::Zero();
    matV = Eigen::Matrix2f::Zero();

    int neighbor_num = point_set.size();
    for(int j = 0; j < neighbor_num; ++j){
        mean[0] += point_set[j].x;
        mean[1] += point_set[j].y;
    }
    mean /= neighbor_num;
    
    Eigen::Matrix2f matA = Eigen::Matrix2f::Zero();
    for(int j = 0; j < neighbor_num; ++j){
        float ax = point_set[j].x - mean[0];
        float ay = point_set[j].y - mean[1];

        matA(0, 0) += ax * ax;
        matA(0, 1) += ax * ay;
        matA(1, 0) += ay * ax;
        matA(1, 1) += ay * ay;
    }
    matA /= neighbor_num;

    // 计算特征值

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> esolver(matA);

    matD = esolver.eigenvalues();
    matV = esolver.eigenvectors();
}