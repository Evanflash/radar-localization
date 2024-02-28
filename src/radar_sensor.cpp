#include "radar_sensor.hpp"

#include <queue>

namespace radar
{

RadarSensor::RadarSensor(const std::string radar_file_path, ll timestamp)
{
    update_radar_data(radar_file_path, timestamp);
}

void RadarSensor::update_radar_data(const std::string radar_file_path, ll timestamp)
{
    this -> timestamp = timestamp;
    std::string path = radar_file_path + "/" + std::to_string(timestamp) + ".png";
    cv::Mat raw_data = cv::imread(path, 0);
    fft_data = cv::Mat::zeros(raw_data.rows, raw_data.cols - 11, CV_32F);
    all_theta.resize(raw_data.rows);
    static const float encode = M_PI / 2800.0;
    for(int i = 0; i < raw_data.rows; ++i){
        uint16_t high_byte = (uint16_t)raw_data.at<uchar>(i, 9);
        uint16_t low_byte = (uint16_t)raw_data.at<uchar>(i, 8);
        all_theta[i]= (float)((high_byte << 8) + low_byte) * encode;
        for(int j = 11; j < raw_data.cols; ++j){
            fft_data.at<float>(i, j - 11) = (float)raw_data.at<uchar>(i, j) / 255.0;
        }
    }
}

void RadarSensor::k_strongest_filter(int k)
{
    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
    for(int i = 0; i < fft_data.rows; ++i){
        // 比较函数
        auto cmp = [&](const int l_ind, const int r_ind){
            return fft_data.at<float>(i, l_ind) > fft_data.at<float>(i, r_ind);
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = 50; j < fft_data.cols; ++j){
            pri_que.push(j);
            while((int)pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            t[i].push_back(cv::Point2f(i, pri_que.top()));
            pri_que.pop();
        }
    }

    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int u = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, u) = t[i][j].x;
            targets(1, u) = t[i][j].y;
            u++;
        }
    }
}

void RadarSensor::scan_denoise(int range, float sigma)
{
    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
    for(int i = 0; i < fft_data.rows; ++i){
        int index = 0;
        float value = 0;
        float mean = 0;
        for(int j = 0; j < fft_data.cols; ++j){
            mean += fft_data.at<float>(i, j);
            if(value < fft_data.at<float>(i, j))
            {
                index = j;
                value = fft_data.at<float>(i, j);
            }   
        }
        mean /= fft_data.cols;
        for(int ind = index; ind >= index - range && ind >= 0; --ind)
        {
            if(fft_data.at<float>(i, ind) > sigma * mean)
                t[i].push_back(cv::Point2f(i, ind));
        }
        for(int ind = index + 1; ind <= index + range && ind < fft_data.cols; ++ind)
        {
            if(fft_data.at<float>(i, ind) > sigma * mean)
                t[i].push_back(cv::Point2f(i, ind));
        }
    }

    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int u = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, u) = t[i][j].x;
            targets(1, u) = t[i][j].y;
            u++;
        }
    }
}

void RadarSensor::motion_compensation(Vec3d relative_pose)
{
    motion.resize(fft_data.rows, Mat3d::Zero());
    for(int i = 0; i < fft_data.rows; ++i){
        motion[i] = pose_to_transformation(((float)(i + 1) / fft_data.rows) * relative_pose);
    }
}

CLOUD::Ptr RadarSensor::get_radar_point_cloud(model md)
{
    CLOUD::Ptr result_point_cloud(new CLOUD());
    result_point_cloud -> reserve(targets.cols());
    std::vector<Mat3d> tmp_motion(fft_data.rows, Mat3d::Identity());
    std::vector<float> tmp_doppler(fft_data.rows, 0);
    if(md == model::motion){
        tmp_motion = this -> motion;
    }else if(md == model::doppler){
        tmp_doppler = this -> doppler;
    }else if(md == model::motion_doppler){
        tmp_motion = this -> motion;
        tmp_doppler = this -> doppler;
    }
    for(int i = 0; i < targets.cols(); ++i){
        double theta = all_theta[targets(0, i)];
        double distance = (targets(1, i) + 0.5) * 0.0438 + tmp_doppler[targets(0, i)];
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        POINT cur_point(distance * cos_theta, -distance * sin_theta, 0, 0);
        transform_point(tmp_motion[targets(0, i)], cur_point);
        result_point_cloud -> push_back(cur_point);
    }

    pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxel_filter -> setInputCloud(result_point_cloud);
    voxel_filter -> setLeafSize(0.1, 0.1, 0.1);
    voxel_filter -> filter(*result_point_cloud);
    return result_point_cloud;
}

Mat3d RadarSensor::pose_to_transformation(Vec3d pose)
{
    double sin_theta = sin(pose[2]);
    double cos_theta = cos(pose[2]);
    Mat3d T;
    T << cos_theta, sin_theta, pose[0],
        -sin_theta, cos_theta, pose[1],
        0, 0, 1;
    return T;
}

void RadarSensor::transform_point(Mat3d &T, POINT &point)
{
    Vec3d p(point.x, point.y, 1);
    p = T * p;
    point.x = p[0];
    point.y = p[1];
}

} // namespace radar