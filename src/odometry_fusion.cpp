#include "odometry_fusion.hpp"

namespace odometry
{

Odometry::Odometry(const std::string radar_data_file_path, const std::string radar_timestamp_file_path,
        const std::string imu_data_file_path, const std::string config_file_path)
    : radar_file_path(radar_data_file_path), 
    config(config_file_path),
    imu_sensor(imu_data_file_path)
{
    read_timestamps(radar_timestamp_file_path);

    // gtsam init
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    // init
    cloud_key_pose_2d.reset(new CLOUD());
}

Odometry::~Odometry()
{

}

void Odometry::laser_cloud_handler()
{
    for(size_t i = 0; i < timestamps.size(); ++i){
        if(i <= 0){

        }
        cur_timestamp = timestamps[i];
        radar_sensor.update_radar_data(radar_file_path, cur_timestamp);
        Vec3d init_pose = obtain_relative_pose(pre_timestamp, cur_timestamp);
        radar_sensor.k_strongest_filter(config.k);
        radar_sensor.motion_compensation(init_pose);
        source_cloud = radar_sensor.get_radar_point_cloud(radar::motion);

        cur_relative_pose = obtain_relative_pose(keyframe_timestamps.back(), cur_timestamp);

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
    std::vector<int> point_search_ind;
    std::vector<float> point_search_dis;

    // create kd-tree
    pcl::KdTreeFLANN<POINT>::Ptr kd_tree(new pcl::KdTreeFLANN<POINT>());
    kd_tree -> setInputCloud(cloud_key_pose_2d);
    Vec3d absolute_pose = keyframe_poses.back() + cur_relative_pose;
    POINT init_pose(absolute_pose[0], absolute_pose[1], 0);
    kd_tree -> radiusSearch(init_pose, config.keyframes_search_radius,
        point_search_ind, point_search_dis);

    surrounding_keyframe_clouds.clear();
    Mat3d base_transformation = pose_to_transformation(keyframe_poses.back());
    base_transformation = base_transformation.inverse();

    for(auto ind : point_search_ind){
        Mat3d cur_T = pose_to_transformation(keyframe_poses[ind]);
        surrounding_keyframe_clouds.push_back(transform_cloud(keyframe_clouds[ind], cur_T));
    }
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

    for(auto point_set : hash_map)
    {
        if((int)point_set.second.size() < least_points_num)
            continue;
        source_feature_set.push_back(point_set.second);
    }
}

void Odometry::scan_to_mulkeframes_optimization()
{
    // 周围信息提取
    divide_into_grid(config.grid_size, config.least_point_num);
    
}

bool Odometry::is_keyframes(ll cur_timestamp)
{
    if(cloud_key_pose_2d -> empty()){
        return true;
    }
    if(cur_timestamp - keyframe_timestamps.back() > config.save_keyframes_time_length){
        return true;
    }
    double dis = sqrt(cur_relative_pose[0] * cur_relative_pose[0] + 
                    cur_relative_pose[1] * cur_relative_pose[1]);
    if(dis < config.save_keyframes_pose_dis && 
        cur_relative_pose[2] < config.save_keyframes_pose_yaw){
        return false;
    }
    return true;
}

void Odometry::save_keyframes_and_factor(ll cur_timestamp)
{
    if(is_keyframes(cur_timestamp) == false){
        return;
    }
    
    // 因子图更新
    add_odom_factor();
    add_gps_factor();
    add_loop_factor();

    // update iSAM
    isam -> update(gtsam_graph, initial_estimate);
    isam -> update();

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
    Vec3d p(point.x, point.y, point.z);
    Vec3d res = pose * p;
    return POINT(p[0], p[1], p[2], point.intensity);
}

CLOUD::Ptr Odometry::transform_cloud(CLOUD::Ptr cloud, Vec3d pose)
{
    Mat3d T = pose_to_transformation(pose);
    CLOUD::Ptr res(new CLOUD());
    res -> reserve(cloud -> size());
    for(auto point : cloud -> points){
        res -> push_back(transform_point(point, T));
    }
    return res;
}

} // namespace odometry