#include "filter.hpp"

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Eigen>


pcl::PointCloud<pcl::PointXYZI>::Ptr extract_surf_point(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, 
    int neighbor_points_num, float distance_max, float linear_max)
{
    struct linear{
        int index;
        float linear_value;
        linear(){}
        linear(int _index, float _linear_value)
            : index(_index), linear_value(_linear_value){}
    };

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_point_cloud(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_point_cloud -> setInputCloud(point_cloud);

    std::vector<linear> point_linear(point_cloud -> size());

    for(int i = 0; i < (int)point_cloud -> size(); ++i){
        std::vector<int> nn_idx(neighbor_points_num + 1);
        std::vector<float> nn_distance(neighbor_points_num + 1);
        kdtree_point_cloud -> nearestKSearch(point_cloud -> points[i], neighbor_points_num, nn_idx, nn_distance);
        if(nn_distance.back() > linear_max){
            point_linear[i] = linear(i, 0);
        } else{
            Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
            for(int i = 1; i <= neighbor_points_num; ++i){
                centroid += Eigen::Vector2d(point_cloud -> points[nn_idx[i]].x, point_cloud -> points[nn_idx[i]].y);
            }
            centroid /= neighbor_points_num;

            Eigen::Matrix2d covraiance = Eigen::Matrix2d::Zero();
            for(int i = 1; i <= neighbor_points_num; ++i){
                Eigen::Vector2d tmp = Eigen::Vector2d(point_cloud -> points[nn_idx[i]].x, 
                    point_cloud -> points[nn_idx[i]].y) - centroid;
                covraiance += tmp * tmp.transpose();
            }
            covraiance /= neighbor_points_num;
            Eigen::Vector2d eigenvalue = covraiance.eigenvalues().real();
            float value_larger = std::max(eigenvalue[0], eigenvalue[1]);
            float value_smaller = std::min(eigenvalue[0], eigenvalue[1]);
            point_linear[i] = linear(i, (value_larger - value_smaller) / (value_smaller + 0.001));
        }
    }

    for(int i = 0; i < (int) point_linear.size(); ++i){
        if(point_linear[i].linear_value > linear_max){
            result_cloud -> push_back(point_cloud -> points[point_linear[i].index]);
        }
    }
    // std::cout << result_cloud -> size() << std::endl;

    return result_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, float least_power)
{
    cv::Mat &fft_data = rd.fft_data; 
    std::vector<spectrum> &spectrum_vec = rd.spectrum_vec;

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < fft_data.rows; ++i){
        auto cmp = [&](const int l_val, const int r_val)
        {
            return fft_data.at<float>(i, l_val) > fft_data.at<float>(i, r_val);
        };

        float sin_theta = sin(spectrum_vec[i].theta);
        float cos_theta = cos(spectrum_vec[i].theta);

        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = 0; j < fft_data.cols; ++j){
            pri_que.push(j);
            while((int)pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            if(fft_data.at<float>(i, pri_que.top()) > least_power){
                float distance = (pri_que.top() + 0.5) * 0.0438;
                result_cloud -> push_back(pcl::PointXYZI(distance * cos_theta, -distance * sin_theta, 
                    0, fft_data.at<float>(i, pri_que.top())));
            }
            pri_que.pop();
        }
    }

    // pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    // voxel_filter -> setInputCloud(result_cloud);
    // voxel_filter -> setLeafSize(0.2, 0.2, 0.2);
    // voxel_filter -> filter(*result_cloud);

    return result_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_filter(radar_data &rd, int k, 
    float least_power, Eigen::Vector3d init_pose)
{
    cv::Mat &fft_data = rd.fft_data; 
    std::vector<spectrum> &spectrum_vec = rd.spectrum_vec;

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < fft_data.rows; ++i){
        // doppler
        // float delta_d = doppler_offset(init_pose[0], init_pose[1], spectrum_vec[i].theta);
        float delta_d = 0;
        auto cmp = [&](const int l_val, const int r_val)
        {
            return fft_data.at<float>(i, l_val) > fft_data.at<float>(i, r_val);
        };

        float sin_theta = sin(spectrum_vec[i].theta);
        float cos_theta = cos(spectrum_vec[i].theta);

        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = 0; j < fft_data.cols; ++j){
            pri_que.push(j);
            while((int)pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            if(fft_data.at<float>(i, pri_que.top()) > least_power){
                float distance = (pri_que.top() + 0.5) * 0.0438 - delta_d;
                // motion distortion
                pcl::PointXYZI point_ori = pcl::PointXYZI(distance * sin_theta, distance * cos_theta, 
                    0, fft_data.at<float>(i, pri_que.top()));
                result_cloud -> push_back(motion_distortion(point_ori, i + 1, init_pose[0], init_pose[1], init_pose[2]));
            }
            pri_que.pop();
        }
    }

    pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    voxel_filter -> setInputCloud(result_cloud);
    voxel_filter -> setLeafSize(0.1, 0.1, 0.1);
    voxel_filter -> filter(*result_cloud);

    return result_cloud;
}

float doppler_offset(float x_offset, float y_offset, float theta)
{
    float beta = 0.049;
    float delta_t = 0.25;
    float v_x = x_offset / delta_t;
    float v_y = y_offset / delta_t;
    float result = beta * (v_y * cos(theta) + v_x * sin(theta));
    return result;
}

pcl::PointXYZI motion_distortion(pcl::PointXYZI point_ori, int ind, float x_offset, float y_offset, float theta)
{
    int num = 400;
    float alpha = 0;//ind * theta / num;
    float x = ind * x_offset / num;
    float y = ind * y_offset / num;
    Eigen::Matrix3d T;
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);
    T << cos_alpha, sin_alpha, x,
        -sin_alpha, cos_alpha, y,
        0, 0, 1;
    Eigen::Vector3d p(point_ori.x, point_ori.y, 1);
    p = T * p;
    return pcl::PointXYZI(p[0], p[1], 0, point_ori.intensity);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr extract_flat_surf_points(
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, float grid_size)
{
    struct point_flat{
        pcl::PointXYZI point;
        float flat;
        point_flat(pcl::PointXYZI _point, float _flat)
            : point(_point), flat(_flat){}
        point_flat(){}
    };

    std::unordered_map<std::string, point_flat> hash_map;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_point_cloud(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_point_cloud -> setInputCloud(point_cloud);

    int neiber_num = 5;
    float max_distance = 3;
    float flat_thres = 1;
    for(auto point : point_cloud -> points){
        std::vector<int> nn_idx(neiber_num);
        std::vector<float> nn_distance(neiber_num);
        kdtree_point_cloud -> nearestKSearch(point, neiber_num, nn_idx, nn_distance);
        if(nn_distance.back() <= max_distance){
            int x_ind = point.x / grid_size;
            int y_ind = point.y / grid_size;
            std::string key = std::to_string(x_ind) + "+" + std::to_string(y_ind);

            // 均值
            float cx = 0, cy = 0;
            for(int j = 0; j < neiber_num; ++j){
                cx += point_cloud -> points[nn_idx[j]].x;
                cy += point_cloud -> points[nn_idx[j]].y;
            }
            cx /= neiber_num;
            cy /= neiber_num;
            // 协方差
            float a11 = 0, a12 = 0, a21 = 0, a22 = 0;
            for(int j = 0; j < neiber_num; ++j){
                float ax = point_cloud -> points[nn_idx[j]].x - cx;
                float ay = point_cloud -> points[nn_idx[j]].y - cy;

                a11 += ax * ax;
                a12 += ax * ay;
                a21 += ay * ax;
                a22 += ay * ay;
            }
            a11 /= neiber_num;
            a12 /= neiber_num;
            a21 /= neiber_num;
            a22 /= neiber_num;

            // 计算特征值
            Eigen::Matrix2d matA = Eigen::Matrix2d::Zero();
            matA(0, 0) = a11;
            matA(0, 1) = a12;
            matA(1, 0) = a21;
            matA(1, 1) = a22;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> esolver(matA);

            Eigen::Matrix<double, 1, 2> matD = esolver.eigenvalues();

           double flat = (matD[1] - matD[0]) / matD[0];

            // 判断是否平坦
            if(flat < flat_thres) continue;

            if(hash_map.find(key) == hash_map.end() || flat > hash_map[key].flat)
                hash_map[key] = point_flat(point, flat);
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(auto f : hash_map){
        result_cloud -> push_back(f.second.point);
    }
    return result_cloud;
}

std::vector<std::vector<pcl::PointXYZI>> divide_into_grid(
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, float grid_size, int least_points_num)
{
    std::unordered_map<std::string, std::vector<pcl::PointXYZI>> hash_map;
    for(auto point : source_cloud -> points){
        int x_ind = point.x / grid_size;
        int y_ind = point.y / grid_size;
        std::string key = std::to_string(x_ind) + "+" + std::to_string(y_ind);
        hash_map[key].push_back(point);
    }

    std::vector<std::vector<pcl::PointXYZI>> result;
    for(auto point_set : hash_map){
        if((int)point_set.second.size() < least_points_num) continue;
        result.push_back(point_set.second);
    }
    
    return result;
}

void cfar1d(cv::Mat fft_data, int window_size, float scale, int guard_cells, int min_range, Eigen::MatrixXd &targets) {
    assert(fft_data.depth() == CV_32F);
    assert(fft_data.channels() == 1);
    auto t1 = std::chrono::high_resolution_clock::now();
    int kernel_size = window_size + guard_cells * 2 + 1;
    cv::Mat kernel = cv::Mat::ones(1, kernel_size, CV_32F) * -1 * scale / window_size;
    kernel.at<float>(0, kernel_size / 2) = 1;
    for (int i = 0; i < guard_cells; i++) {
        kernel.at<float>(0, window_size / 2 + i) = 0;
    }
    for (int i = 0; i < guard_cells; i++) {
        kernel.at<float>(0, kernel_size / 2 + 1 + i) = 0;
    }
    cv::Mat output;
    cv::filter2D(fft_data, output, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);
    // Find filter responses > 0
    std::vector<cv::Point2f> t;
    for (int i = 0; i < output.rows; ++i) {
        for (int j = min_range; j < output.cols; j++) {
            if (output.at<float>(i, j) > 0) {
                t.push_back(cv::Point(i, j));
            }
        }
    }
    targets = Eigen::MatrixXd::Ones(3, t.size());
    for (uint i = 0; i < t.size(); ++i) {
        targets(0, i) = t[i].x;
        targets(1, i) = t[i].y;
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    std::cout << "feature extraction: " << e.count() << std::endl;
}

// Runtime: 0.035s
double cen2018features(cv::Mat fft_data, float zq, int sigma_gauss, int min_range, Eigen::MatrixXd &targets) {
    auto t1 = std::chrono::high_resolution_clock::now();

    std::vector<float> sigma_q(fft_data.rows, 0);
    // Estimate the bias and subtract it from the signal
    cv::Mat q = fft_data.clone();
    for (int i = 0; i < fft_data.rows; ++i) {
        float mean = 0;
        for (int j = 0; j < fft_data.cols; ++j) {
            mean += fft_data.at<float>(i, j);
        }
        mean /= fft_data.cols;
        for (int j = 0; j < fft_data.cols; ++j) {
            q.at<float>(i, j) = fft_data.at<float>(i, j) - mean;
        }
    }

    // Create 1D Gaussian Filter (0.09)
    assert(sigma_gauss % 2 == 1);
    int fsize = sigma_gauss * 3;
    int mu = fsize / 2;
    float sig_sqr = sigma_gauss * sigma_gauss;
    cv::Mat filter = cv::Mat::zeros(1, fsize, CV_32F);
    float s = 0;
    for (int i = 0; i < fsize; ++i) {
        filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
        s += filter.at<float>(0, i);
    }
    filter /= s;
    cv::Mat p;
    cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

    // Estimate variance of noise at each azimuth (0.004)
    for (int i = 0; i < fft_data.rows; ++i) {
        int nonzero = 0;
        for (int j = 0; j < fft_data.cols; ++j) {
            float n = q.at<float>(i, j);
            if (n < 0) {
                sigma_q[i] += 2 * (n * n);
                nonzero++;
            }
        }
        if (nonzero)
            sigma_q[i] = sqrt(sigma_q[i] / nonzero);
        else
            sigma_q[i] = 0.034;
    }

    // Extract peak centers from each azimuth
    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
#pragma omp parallel for
    for (int i = 0; i < fft_data.rows; ++i) {
        std::vector<int> peak_points;
        float thres = zq * sigma_q[i];
        for (int j = min_range; j < fft_data.cols; ++j) {
            float nqp = exp(-0.5 * pow((q.at<float>(i, j) - p.at<float>(i, j)) / sigma_q[i], 2));
            float npp = exp(-0.5 * pow(p.at<float>(i, j) / sigma_q[i], 2));
            float b = nqp - npp;
            float y = q.at<float>(i, j) * (1 - nqp) + p.at<float>(i, j) * b;
            if (y > thres) {
                peak_points.push_back(j);
            } else if (peak_points.size() > 0) {
                t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
                peak_points.clear();
            }
        }
        if (peak_points.size() > 0)
            t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
    }

    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int k = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, k) = t[i][j].x;
            targets(1, k) = t[i][j].y;
            k++;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}

struct Point {
    float i;
    int a;
    int r;
    Point(float i_, int a_, int r_) {i = i_; a = a_; r = r_;}
};

struct greater_than_pt {
    inline bool operator() (const Point& p1, const Point& p2) {
        return p1.i > p2.i;
    }
};

static void findRangeBoundaries(cv::Mat &s, int a, int r, int &rlow, int &rhigh) {
    rlow = r;
    rhigh = r;
    if (r > 0) {
        for (int i = r - 1; i >= 0; i--) {
            if (s.at<float>(a, i) < 0)
                rlow = i;
            else
                break;
        }
    }
    if (r < s.rows - 1) {
        for (int i = r + 1; i < s.cols; i++) {
            if (s.at<float>(a, i) < 0)
                rhigh = i;
            else
                break;
        }
    }
}

static bool checkAdjacentMarked(cv::Mat &R, int a, int start, int end) {
    int below = a - 1;
    int above = a + 1;
    if (below < 0)
        below = R.rows - 1;
    if (above >= R.rows)
        above = 0;
    for (int r = start; r <= end; r++) {
        if (R.at<float>(below, r) || R.at<float>(above, r))
            return true;
    }
    return false;
}

static void getMaxInRegion(cv::Mat &h, int a, int start, int end, int &max_r) {
    int max = -1000;
    for (int r = start; r <= end; r++) {
        if (h.at<float>(a, r) > max) {
            max = h.at<float>(a, r);
            max_r = r;
        }
    }
}

// Runtime: 0.050s
double cen2019features(cv::Mat fft_data, int max_points, int min_range, Eigen::MatrixXd &targets) {
    auto t1 = std::chrono::high_resolution_clock::now();
    // Calculate gradient along each azimuth using the Prewitt operator
    cv::Mat prewitt = cv::Mat::zeros(1, 3, CV_32F);
    prewitt.at<float>(0, 0) = -1;
    prewitt.at<float>(0, 2) = 1;
    cv::Mat g;
    cv::filter2D(fft_data, g, -1, prewitt, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);
    g = cv::abs(g);
    double maxg = 1, ming = 1;
    cv::minMaxIdx(g, &ming, &maxg);
    g /= maxg;

    // Subtract the mean from the radar data and scale it by 1 - gradient magnitude
    float mean = cv::mean(fft_data)[0];
    cv::Mat s = fft_data - mean;
    cv::Mat h = s.mul(1 - g);
    float mean_h = cv::mean(h)[0];

    // Get indices in descending order of intensity
    std::vector<Point> vec;
    for (int i = 0; i < fft_data.rows; ++i) {
        for (int j = 0; j < fft_data.cols; ++j) {
            if (h.at<float>(i, j) > mean_h)
                vec.push_back(Point(h.at<float>(i, j), i, j));
        }
    }
    std::sort(vec.begin(), vec.end(), greater_than_pt());

    // Create a matrix, R, of "marked" regions consisting of continuous regions of an azimuth that may contain a target
    int false_count = fft_data.rows * fft_data.cols;
    uint j = 0;
    int l = 0;
    cv::Mat R = cv::Mat::zeros(fft_data.rows, fft_data.cols, CV_32F);
    while (l < max_points && j < vec.size() && false_count > 0) {
        if (!R.at<float>(vec[j].a, vec[j].r)) {
            int rlow = vec[j].r;
            int rhigh = vec[j].r;
            findRangeBoundaries(s, vec[j].a, vec[j].r, rlow, rhigh);
            bool already_marked = false;
            for (int i = rlow; i <= rhigh; i++) {
                if (R.at<float>(vec[j].a, i)) {
                    already_marked = true;
                    continue;
                }
                R.at<float>(vec[j].a, i) = 1;
                false_count--;
            }
            if (!already_marked)
                l++;
        }
        j++;
    }

    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);

#pragma omp parallel for
    for (int i = 0; i < fft_data.rows; i++) {
        // Find the continuous marked regions in each azimuth
        int start = 0;
        int end = 0;
        bool counting = false;
        for (int j = min_range; j < fft_data.cols; j++) {
            if (R.at<float>(i, j)) {
                if (!counting) {
                    start = j;
                    end = j;
                    counting = true;
                } else {
                    end = j;
                }
            } else if (counting) {
                // Check whether adjacent azimuths contain a marked pixel in this range region
                if (checkAdjacentMarked(R, i, start, end)) {
                    int max_r = start;
                    getMaxInRegion(h, i, start, end, max_r);
                    t[i].push_back(cv::Point(i, max_r));
                }
                counting = false;
            }
        }
    }

    int size = 0;
    for (uint i = 0; i < t.size(); ++i) {
        size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int k = 0;
    for (uint i = 0; i < t.size(); ++i) {
        for (uint j = 0; j < t[i].size(); ++j) {
            targets(0, k) = t[i][j].x;
            targets(1, k) = t[i][j].y;
            k++;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}