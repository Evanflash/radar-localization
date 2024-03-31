#include "odometry_fusion.hpp"

namespace odometry
{

Odometry::Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path, const std::string gps_data_file_path)
    : radar_file_path(radar_data_file_path), 
    imu_sensor(imu_data_file_path),
    gps_sensor(gps_data_file_path)
{
    read_timestamps(radar_timestamp_file_path);

    // gtsam init
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    parameters.factorization = gtsam::ISAM2Params::QR;
    isam = new gtsam::ISAM2(parameters);

    // init
    cloud_key_pose_2d.reset(new CLOUD());

    // 超参数
    k = 12;
    keyframes_search_radius = 10;
    save_keyframes_time_length = 1000000;
    save_keyframes_pose_dis = 1;
    save_keyframes_pose_yaw = 0.05;
    neighbor_num = 5;
    iterations = 10;
    grid_size = 2;
    least_point_num = 5;
    save_file_path = "/home/evan/code/radar-localization/test/result/0110/0110_mdad_thres_mul.txt";

    aLoopIsClosed = false;
    last_keyframe_index = 0;
    poseCovThreshold = 16;
}

Odometry::~Odometry()
{

}

void Odometry::laser_cloud_handler()
{
    Vec3d last_relative_pose(2.4, 0, 0);
    for(size_t i = 0; i < timestamps.size(); ++i){
        if(i <= 0){
            cur_timestamp = timestamps[i];
            radar_sensor.update_radar_data(radar_file_path, cur_timestamp);
            // radar_sensor.k_strongest_filter(k);
            radar_sensor.scan_denoise(5, 7);
            source_cloud = radar_sensor.get_radar_point_cloud(radar::motion);
            cur_relative_pose = Vec3d::Zero();
            last_keyframe_pose = Vec3d::Zero();
            save_keyframes_and_factor();
        }else{
            cur_timestamp = timestamps[i];
            radar_sensor.update_radar_data(radar_file_path, cur_timestamp);
            Vec3d init_pose = obtain_relative_pose(pre_timestamp, cur_timestamp);
            // 滤波方案
            // radar_sensor.k_strongest_filter(k);
            radar_sensor.scan_denoise(5, 7);
            radar_sensor.motion_compensation(init_pose);
            // radar_sensor.motion_compensation(last_relative_pose);
            source_cloud = radar_sensor.get_radar_point_cloud(radar::motion);
            
            // cur_relative_pose = sum_pose(last_keyframe_pose, last_relative_pose);
            cur_relative_pose = obtain_relative_pose(keyframe_timestamps.back(), cur_timestamp);    
            // std::cout << "-----------------------" << std::endl;
            // std::cout << "init" << std::endl;
            // std::cout << cur_relative_pose[0] << " " << cur_relative_pose[1] << " " <<
            //     cur_relative_pose[2] << std::endl;
            extract_surrounding_keyframes();

            scan_to_mulkeframes_optimization();
            
            last_relative_pose = sub_pose(cur_relative_pose, last_keyframe_pose);
            last_keyframe_pose = cur_relative_pose;

            save_keyframes_and_factor();
            // std::cout << "final" << std::endl;
            // std::cout << cur_relative_pose[0] << " " << cur_relative_pose[1] << " " <<
            //     cur_relative_pose[2] << std::endl;
        }
        correct_poses();
        pre_timestamp = cur_timestamp;
    }
}

void Odometry::read_timestamps(const std::string radar_timestamp_file_path)
{
    std::fstream input_file(radar_timestamp_file_path.c_str(), std::ios::in);
    std::string line;
    while(std::getline(input_file, line)){
        std::stringstream ss(line);
        std::string str;
        std::getline(ss, str, ' ');
        long long timestamp = std::stoll(str);
        timestamps.push_back(timestamp);
    }
    input_file.close();
}

Vec3d Odometry::obtain_relative_pose(ll pre_timestamp, ll nxt_timestamp)
{
    return imu_sensor.get_relative_pose(pre_timestamp, nxt_timestamp);
}

void Odometry::extract_surrounding_keyframes()
{
    // 时间方面最近值
    Mat3d base_transformation = pose_to_transformation(keyframe_poses.back());
    base_transformation = base_transformation.inverse().eval();
    surrounding_keyframe_clouds.clear();
    int size = keyframe_poses.size();
    for(int i = size - 1; i > size - 3 && i >= 0; --i){
        Mat3d cur_T = base_transformation * pose_to_transformation(keyframe_poses[i]);
        surrounding_keyframe_clouds.push_back(transform_cloud(keyframe_clouds[i], cur_T));
    }


    // std::vector<int> point_search_ind;
    // std::vector<float> point_search_dis;
    // // // create kd-tree
    // pcl::KdTreeFLANN<POINT>::Ptr kd_tree(new pcl::KdTreeFLANN<POINT>());
    // kd_tree -> setInputCloud(cloud_key_pose_2d);
    // Vec3d absolute_pose = sum_pose(keyframe_poses.back(), cur_relative_pose);
    // POINT init_pose(absolute_pose[0], absolute_pose[1], 0);
    // kd_tree -> radiusSearch(init_pose, keyframes_search_radius,
    //     point_search_ind, point_search_dis);

    // surrounding_keyframe_clouds.clear();
    // Mat3d base_transformation = pose_to_transformation(keyframe_poses.back());
    // base_transformation = base_transformation.inverse().eval();

    // CLOUD::Ptr show(new CLOUD());
    // for(auto ind : point_search_ind){
    //     Mat3d cur_T = base_transformation * pose_to_transformation(keyframe_poses[ind]);
    //     // Mat3d cur_T = pose_to_transformation(keyframe_poses[ind] - keyframe_poses.back());
    //     surrounding_keyframe_clouds.push_back(transform_cloud(keyframe_clouds[ind], cur_T));
    //     *show += *surrounding_keyframe_clouds.back();
    // }
    // std::cout << surrounding_keyframe_clouds.size() << std::endl;
    // cv::Mat n_image = pointcloud_to_cartesian_points(show, 800, 800, 0.2);
    // cv::Mat m_image = pointcloud_to_cartesian_points(source_cloud, 800, 800, 0.2);
    // cv::Mat image(800, 1600, CV_8U);
    // n_image.copyTo(image.colRange(0, 800));
    // m_image.copyTo(image.colRange(800, 1600));
    // cv::imshow("", image);
    // cv::waitKey(0);
}

void Odometry::divide_into_grid(float grid_size, int least_points_num)
{
    std::unordered_map<std::string, std::vector<POINT>> hash_map;
    for(auto point : source_cloud -> points)
    {
        int x_ind = point.x / grid_size;
        int y_ind = point.y / grid_size;
        std::string key = std::to_string(x_ind) + "+" + std::to_string(y_ind);
        hash_map[key].push_back(point);
    }

    source_feature_set.clear();
    for(auto point_set : hash_map)
    {
        if((int)point_set.second.size() < least_points_num)
            continue;
        source_feature_set.push_back(point_set.second);
    }
}

void Odometry::calculate_mean_and_cov(std::vector<POINT> point_set, Eigen::Vector2d& mean, 
        Eigen::Matrix<double, 1, 2>& matD, Eigen::Matrix2d& matV)
{
    mean = Eigen::Vector2d::Zero();
    matD = Eigen::Matrix<double, 1, 2>::Zero();
    matV = Eigen::Matrix2d::Zero();

    int neighbor_num = point_set.size();
    for(int j = 0; j < neighbor_num; ++j){
        mean[0] += point_set[j].x;
        mean[1] += point_set[j].y;
    }
    mean /= neighbor_num;
    
    Eigen::Matrix2d matA = Eigen::Matrix2d::Zero();
    for(int j = 0; j < neighbor_num; ++j){
        double ax = point_set[j].x - mean[0];
        double ay = point_set[j].y - mean[1];

        matA(0, 0) += ax * ax;
        matA(0, 1) += ax * ay;
        matA(1, 0) += ay * ax;
        matA(1, 1) += ay * ay;
    }
    matA /= neighbor_num;

    // 计算特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> esolver(matA);
    matD = esolver.eigenvalues();
    matV = esolver.eigenvectors();
}

void Odometry::scan_to_mulkeframes_optimization()
{   
    float cost = 0, last_cost = 0;
    // 周围信息提取
    divide_into_grid(grid_size, least_point_num);

    // 高斯牛顿法
    for(int iterCount = 0; iterCount < iterations; iterCount++)
    {
        Mat3d H = Mat3d::Zero();
        Vec3d B = Vec3d::Zero();

        cost = 0;
        int nums = 0;

        // 生成新的特征点
        std::vector<std::vector<POINT>> tmp_point_set = source_feature_set;
        for(size_t i = 0; i < tmp_point_set.size(); ++i){
            for(size_t j = 0; j < tmp_point_set[i].size(); ++j){
                tmp_point_set[i][j] = transform_point(tmp_point_set[i][j], cur_relative_pose);
            }
        }

        for(auto target_cloud : surrounding_keyframe_clouds)
        {
            pcl::KdTreeFLANN<POINT>::Ptr kdtree_surf_points(new pcl::KdTreeFLANN<POINT>());
            kdtree_surf_points -> setInputCloud(target_cloud);

            // 遍历source特征点
            for(size_t i = 0; i < tmp_point_set.size(); ++i)
            {
                Eigen::Vector2d source_ori;
                Eigen::Vector2d source_mean;
                Eigen::Matrix<double, 1, 2> source_matD;
                Eigen::Matrix2d source_matV;
                calculate_mean_and_cov(source_feature_set[i], source_ori, source_matD, source_matV);
                calculate_mean_and_cov(tmp_point_set[i], source_mean, source_matD, source_matV);

                POINT point_ori = POINT(source_ori[0], source_ori[1], 0, 0);
                POINT point_trans = POINT(source_mean[0], source_mean[1], 0, 0);

                std::vector<int> nn_idx(neighbor_num);
                std::vector<float> nn_distance(neighbor_num);
                
                kdtree_surf_points -> nearestKSearch(point_trans, neighbor_num, nn_idx, nn_distance);
                if(nn_distance.back() < 5.0)
                {
                    std::vector<POINT> target_points_set;
                    for(auto ind : nn_idx){
                        target_points_set.push_back(target_cloud -> points[ind]);
                    }
                    Eigen::Vector2d target_mean;
                    Eigen::Matrix<double, 1, 2> target_matD;
                    Eigen::Matrix2d target_matV;
                    calculate_mean_and_cov(target_points_set, target_mean, target_matD, target_matV);

                    if(target_matD[1] > 2 * target_matD[0])
                    {
                        double x0 = point_trans.x;
                        double y0 = point_trans.y;

                        double weight = sqrt(abs(target_matV(0, 0) * source_matV(0, 0) + 
                            target_matV(1, 0) * source_matV(1, 0)));
                    
                        double x1 = target_mean[0] + weight * target_matV(0, 1);
                        double y1 = target_mean[1] + weight * target_matV(1, 1);
                        double x2 = target_mean[0] - weight * target_matV(0, 1);
                        double y2 = target_mean[1] - weight * target_matV(1, 1);

                        double a = sqrt((x0 - x1) * (x0 - x1) +
                                        (y0 - y1) * (y0 - y1));
                        double b = sqrt((x0 - x2) * (x0 - x2) +
                                        (y0 - y2) * (y0 - y2));
                        double c = sqrt((x1 - x2) * (x1 - x2) +
                                        (y1 - y2) * (y1 - y2));
                        
                        // 海伦公式计算面积
                        double l = (a + b + c) / 2;
                        double S = sqrt(l * (l - a) * (l - b) * (l - c));

                        double error = S;

                        // 计算雅克比矩阵
                        double theta = cur_relative_pose[2];
                        Eigen::Vector3d dx0(1, 0, 
                            -point_ori.x * sin(theta) + point_ori.y * cos(theta));
                        Eigen::Vector3d dy0(0, 1,
                            -point_ori.x * cos(theta) - point_ori.y * sin(theta));

                        Eigen::Vector3d da = (1 / a) * 
                            ((x0 - x1) * dx0 + (y0 - y1) * dy0);
                        Eigen::Vector3d db = (1 / b) *
                            ((x0 - x2) * dx0 + (y0 - y2) * dy0);

                        Eigen::Vector3d J = (1 / (8 * S)) * 
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
        Eigen::Vector3d dx = H.ldlt().solve(B);
        if(std::isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }

        cost = cost / nums;
        // std::cout << "  *iterator" << iterCount + 1 << "*" << std::endl;
        // std::cout << "last_cost = " << last_cost << std::endl;
        // std::cout << "   cost   = " << cost << std::endl;
        // std::cout << "nums = " << nums << std::endl;
        if(iterCount > 0 && last_cost < cost) break;

        cur_relative_pose = cur_relative_pose + dx;
        last_cost = cost;
    }
}

bool Odometry::is_keyframes()
{
    if(cloud_key_pose_2d -> empty()){
        return true;
    }
    // if(cur_timestamp - keyframe_timestamps.back() > save_keyframes_time_length){
    //     return true;
    // }
    double dis = sqrt(cur_relative_pose[0] * cur_relative_pose[0] + 
                    cur_relative_pose[1] * cur_relative_pose[1]);
    if(dis < save_keyframes_pose_dis && 
        abs(cur_relative_pose[2]) < save_keyframes_pose_yaw){
        return false;
    }
    return true;
}

void Odometry::add_odom_factor()
{
    if(cloud_key_pose_2d -> empty()){
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = 
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished());
        gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, vec_to_gtsam_pose(relative_to_absolute_pose(cur_relative_pose)), prior_noise));
        initial_estimate.insert(0, vec_to_gtsam_pose(relative_to_absolute_pose(cur_relative_pose)));
    }else{
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise = 
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-8, 1e-8, 1e-4, 1e-2, 1e-2, 1e-8).finished());
        gtsam::Pose3 pose_from = vec_to_gtsam_pose(keyframe_poses.back());
        gtsam::Pose3 pose_to = vec_to_gtsam_pose(relative_to_absolute_pose(cur_relative_pose));
        gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloud_key_pose_2d -> size() - 1, cloud_key_pose_2d -> size(), pose_from.between(pose_to), odom_noise
        ));
        // gtsam::Pose2 between = pose_from.between(pose_to);
        // std::cout << between.translation().x() << " " << between.translation().y() << " " <<
        //     between.rotation().theta() << std::endl;
        // std::cout << cur_relative_pose[0] << " " << cur_relative_pose[1] << " " <<
        //     cur_relative_pose[2] << std::endl;
        // gtsam::Pose2 pose_from = vec_to_gtsam_pose(imu_sensor.get_imu_data_by_timestamp(keyframe_timestamps.back()));
        // gtsam::Pose2 pose_to0 = vec_to_gtsam_pose(imu_sensor.get_imu_data_by_timestamp(cur_timestamp));
        // gtsam::Pose2 between = pose_from.between(pose_to0);
        // Vec3d relative = imu_sensor.get_relative_pose(keyframe_timestamps.back(), cur_timestamp);
        // std::cout << between.translation().x() << " " << between.translation().y() << " " <<
        //     between.rotation().theta() << std::endl;
        // std::cout << relative[0] << " " << relative[1] << " " <<
        //     relative[2] << std::endl;
        // gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        //     cloud_key_pose_2d -> size() - 1, cloud_key_pose_2d -> size(), 
        //     vec_to_gtsam_pose(imu_sensor.get_relative_pose(keyframe_timestamps.back(), cur_timestamp)), odom_noise
        // ));
        initial_estimate.insert(cloud_key_pose_2d -> size(), pose_to);
    }
}

void Odometry::add_gps_factor()
{
    if((int)cloud_key_pose_2d -> size() - last_keyframe_index < 5)
        return;
    if(poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
        return;
    

    gtsam::noiseModel::Diagonal::shared_ptr gps_noise = 
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1, 1, 1e-8).finished());
    
    Vec2d between = gps_sensor.get_gps_data_by_timestamp(cur_timestamp);
    if(gps_sensor.is_bad_gps_data())
        return;
    
    gtsam::GPSFactor gps_factor(
        cloud_key_pose_2d -> size(),
        gtsam::Point3(between[0], between[1], 0),
        gps_noise
    );
    gtsam_graph.add(gps_factor);
    aLoopIsClosed = true;
    last_keyframe_index = cloud_key_pose_2d -> size();


    // Vec3d between = imu_sensor.get_relative_pose(keyframe_timestamps.front(), cur_timestamp);
    // if(imu_sensor.gps_data_is_bad())
    //     return;
    // // std::cout << between[0] << " " << between[1] << " " << between[2] << std::endl;
    // double sin_theta = sin(between[2]);
    // double cos_theta = cos(between[2]);
    // gtsam::GPSFactor gps_factor(
    //     cloud_key_pose_2d -> size(), 
    //     gtsam::Point3(between[0] + 0.86 * cos_theta, between[1] - 0.86 * sin_theta, 0), gps_noise);
    // gtsam_graph.add(gps_factor);
    // aLoopIsClosed = true;
    // last_keyframe_index = cloud_key_pose_2d -> size();
    
    // aLoopIsClosed = true;
}

void Odometry::save_keyframes_and_factor()
{
    if(is_keyframes() == false){
        return;
    }
    
    // 因子图更新
    add_odom_factor();
    // add_gps_factor();
    // add_loop_factor();

    // update iSAM
    isam -> update(gtsam_graph, initial_estimate);
    isam -> update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtsam_graph.resize(0);
    initial_estimate.clear();

    // save
    POINT cur_pose_2d;
    Vec3d cur_absolute_pose;
    gtsam::Pose3 latest_estimate;

    isam_current_estimate = isam -> calculateEstimate();
    latest_estimate = isam_current_estimate.at<gtsam::Pose3>(isam_current_estimate.size() - 1);

    // 更新关键帧点云
    cur_pose_2d.x = latest_estimate.translation().x();
    cur_pose_2d.y = latest_estimate.translation().y();
    cur_pose_2d.z = 0;
    cur_pose_2d.intensity = cloud_key_pose_2d -> size();
    cloud_key_pose_2d -> push_back(cur_pose_2d);

    // 更新关键帧位姿
    cur_absolute_pose[0] = cur_pose_2d.x;
    cur_absolute_pose[1] = cur_pose_2d.y;
    cur_absolute_pose[2] = -latest_estimate.rotation().yaw();

    // 更新状态
    keyframe_clouds.push_back(source_cloud);
    keyframe_poses.push_back(cur_absolute_pose);
    keyframe_timestamps.push_back(cur_timestamp);

    last_keyframe_pose = Vec3d::Zero();

    // 协方差
    poseCovariance = isam -> marginalCovariance(isam_current_estimate.size() - 1);
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << poseCovariance << std::endl;
    // // 保存
    save_path(save_file_path);
}

void Odometry::correct_poses()
{
    if(cloud_key_pose_2d -> empty() || aLoopIsClosed == false)
        return;
    
    int num_poses = isam_current_estimate.size();

    for(int i = 0; i < num_poses; ++i){
        cloud_key_pose_2d -> points[i].x = isam_current_estimate.at<gtsam::Pose3>(i).translation().x();
        cloud_key_pose_2d -> points[i].y = isam_current_estimate.at<gtsam::Pose3>(i).translation().y();

        keyframe_poses[i][0] = cloud_key_pose_2d -> points[i].x;
        keyframe_poses[i][1] = cloud_key_pose_2d -> points[i].y;
        keyframe_poses[i][2] = -isam_current_estimate.at<gtsam::Pose3>(i).rotation().yaw();
    }

    aLoopIsClosed = false;
    save_path(save_file_path);
}

void Odometry::save_path(const std::string save_file_path)
{
    // std::fstream output(save_file_path.c_str(), std::ios::out | std::ios::app);
    // output << std::to_string(keyframe_timestamps.back()) << " " << 
    //     std::to_string(keyframe_poses.back()[0]) << " " << 
    //     std::to_string(keyframe_poses.back()[1]) << " " <<
    //     std::to_string(keyframe_poses.back()[2]) << " " << std::endl;
    std::fstream output(save_file_path.c_str(), std::ios::out);
    for(size_t i = 0; i < keyframe_timestamps.size(); ++i){
        output << std::to_string(keyframe_timestamps[i]) << " " << 
        std::to_string(keyframe_poses[i][0]) << " " << 
        std::to_string(keyframe_poses[i][1]) << " " <<
        std::to_string(keyframe_poses[i][2]) << " " << std::endl;
    }
    output.close();
}

Vec3d Odometry::relative_to_absolute_pose(Vec3d pose)
{
    if(keyframe_poses.empty()){
        return pose;
    }else{
        Mat3d pre_absolute_transformation = pose_to_transformation(keyframe_poses.back());
        Mat3d cur_relative_transformation = pose_to_transformation(pose);
        Mat3d cur_absolute_transformation = pre_absolute_transformation * cur_relative_transformation;
        Vec3d cur_absolute_pose;
        cur_absolute_pose[0] = cur_absolute_transformation(0, 2);
        cur_absolute_pose[1] = cur_absolute_transformation(1, 2);
        cur_absolute_pose[2] = pose[2] + keyframe_poses.back()[2];
        return cur_absolute_pose;
    }
}

gtsam::Pose3 Odometry::vec_to_gtsam_pose(Vec3d pose)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, -pose[2]), gtsam::Point3(pose[0], pose[1], 0));
}

Mat3d Odometry::pose_to_transformation(Vec3d pose)
{
    Mat3d T;
    double sin_theta = sin(pose[2]);
    double cos_theta = cos(pose[2]);
    T << cos_theta, sin_theta, pose[0],
        -sin_theta, cos_theta, pose[1],
        0, 0, 1;
    return T;
}

POINT Odometry::transform_point(POINT point, Vec3d pose)
{
    Mat3d T = pose_to_transformation(pose);
    return transform_point(point, T);
}

POINT Odometry::transform_point(POINT point, Mat3d pose)
{
    Vec3d p(point.x, point.y, 1);
    p = pose * p;
    return POINT(p[0], p[1], 0, point.intensity);
}

CLOUD::Ptr Odometry::transform_cloud(CLOUD::Ptr cloud, Mat3d T)
{
    CLOUD::Ptr res(new CLOUD());
    res -> reserve(cloud -> size());
    for(auto point : cloud -> points){
        res -> push_back(transform_point(point, T));
    }
    return res;
}

Vec3d Odometry::sum_pose(Vec3d absolute, Vec3d relative)
{
    Mat3d a = pose_to_transformation(absolute);
    Mat3d r = pose_to_transformation(relative);
    Mat3d tmp = a * r;
    double theta = absolute[2] + relative[2];
    while (theta >= 2 * M_PI)
    {
        theta -= 2 * M_PI;
    }
    while(theta <= -2 * M_PI){
        theta += 2 * M_PI;
    }
    Vec3d result(tmp(0, 2), tmp(1, 2), theta);
    return result;
}

Vec3d Odometry::sub_pose(Vec3d first, Vec3d second)
{
    Mat3d result_T = pose_to_transformation(second).inverse() * pose_to_transformation(first);
    double theta = first[2] - second[2];
    while (theta >= 2 * M_PI)
    {
        theta -= 2 * M_PI;
    }
    while(theta <= -2 * M_PI){
        theta += 2 * M_PI;
    }
    return Vec3d(result_T(0, 2), result_T(1, 2), theta);
}

} // namespace odometry