#include "features.hpp"

#include <chrono>
#include <iostream>
#include <queue>
#include <fstream>
#include <algorithm>


#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

double cen_2018_features(cv::Mat fft_data, float zq, int sigma_gauss, int min_range, Eigen::MatrixXd &targets)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    std::string file_name;
    int num;
    std::fstream input_file("/home/evan/code/radar-localization/log/spectrum/spectrum_config.txt",
            std::ios::in);
    std::string line;
    std::getline(input_file, line);
    file_name = line;
    std::getline(input_file, line);
    num = std::stoi(line);
    input_file.close();

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
    std::fstream without_bias("/home/evan/code/radar-localization/log/spectrum/" 
            + file_name + "_withoutbias.txt",
            std::ios::out);
    for(int j = 0; j < fft_data.cols; ++j){
        without_bias << q.at<float>(num, j) << std::endl;
    }
    without_bias.close();

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

    std::fstream after_filter("/home/evan/code/radar-localization/log/spectrum/" 
            + file_name + "_afterfilter.txt",
            std::ios::out);
    for(int j = 0; j < fft_data.cols; ++j){
        after_filter << p.at<float>(num, j) << std::endl;
    }
    after_filter.close();


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
    /*
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
    */
    uint k = 12;
    int size = 0;
    for(uint i = 0; i < t.size(); ++i){
        if(t[i].size() > k) size += k;
        else size += t[i].size();
    }
    targets = Eigen::MatrixXd::Ones(3, size);
    int h = 0;
    for(uint i = 0; i < t.size(); ++i){
        auto cmp = [&](uint l_val, uint r_val){
            return fft_data.at<float>(t[i][l_val].x, t[i][l_val].y) > 
                fft_data.at<float>(t[i][r_val].x, t[i][r_val].y); 
        };
        std::priority_queue<uint, std::vector<uint>, decltype(cmp)> pri_que(cmp);
        for(uint j = 0; j < t[i].size(); ++j){
            pri_que.push(j);
            while(pri_que.size() > k) pri_que.pop();
        }
        while(!pri_que.empty()){
            targets(0, h) = t[i][pri_que.top()].x;
            targets(1, h) = t[i][pri_que.top()].y;
            h++;
            pri_que.pop();
        }
    }


    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}

double k_strongest_features(cv::Mat fft_data, int min_range, int k, Eigen::MatrixXd &targets)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
    for(int i = 0; i < fft_data.rows; ++i){
        // 比较函数
        auto cmp = [&](const int l_ind, const int r_ind){
            return fft_data.at<float>(i, l_ind) > fft_data.at<float>(i, r_ind);
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);
        for(int j = min_range; j < fft_data.cols; ++j){
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

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}


double my_features(cv::Mat fft_data, Eigen::MatrixXd &targets)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
    for(int i = 0; i < fft_data.rows; ++i){
        int beginning = 0;

        float all_mean = 0;
        for(int j = 0; j < fft_data.cols; ++j){
            all_mean += fft_data.at<float>(i, j);
        }
        all_mean = all_mean / fft_data.cols;

        float mean = 0;
        for(int j = beginning; j < fft_data.cols; ++j){
            mean += fft_data.at<float>(i, j);
        }
        mean = mean / (fft_data.cols - beginning);

        float delta = 0;
        for(int j = beginning; j < fft_data.cols; ++j){
            delta += (fft_data.at<float>(i, j) - mean) * (fft_data.at<float>(i, j) - mean);
        }
        delta = sqrt(delta / (fft_data.cols - beginning));

        // float thres = all_mean + mean + 3 * delta;
        float thres = mean + 3 * delta;

        auto cmp = [&](const int l_ind, const int r_ind){
            return fft_data.at<float>(i, l_ind) > fft_data.at<float>(i, r_ind);
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> pri_que(cmp);

        for(int j = 0; j < fft_data.cols; ++j){
            if(fft_data.at<float>(i, j) > thres){
                // t[i].push_back(cv::Point2f(i, j));
                
                pri_que.push(j);
                while((int)pri_que.size() > 100) pri_que.pop();
            }
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
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e = t2 - t1;
    return e.count();
}
