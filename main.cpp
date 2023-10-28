#include <iostream>
#include <fstream>

#include "features.hpp"
#include "radar_utils.hpp"
#include "registration.hpp"
#include "test_features.cpp"

void read_config_cen(int &zq, int &sigma, const std::string file_path)
{
    std::fstream input(file_path.c_str(), std::ios::in);
    std::string line;
    getline(input, line);
    zq = std::stoi(line);
    getline(input, line);
    sigma = std::stoi(line);
    input.close();
}

void test(){
    radar_data rd1;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131046353776", rd1);
    // k_strongest_features(rd1.fft_data.clone(), 58, 12, rd1.targets);
    radar_data rd2;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131046606586", rd2);
    // k_strongest_features(rd2.fft_data.clone(), 58, 12, rd2.targets);
    // compare_two_filter(rd1, rd2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // targets_to_point_cloud(rd1, source_cloud);
    // targets_to_point_cloud(rd2, target_cloud);
    // std::cout << pcl_icp_registration(source_cloud, target_cloud, 30) << std::endl;
    // std::cout << "-------------------------------" << std::endl;
    int zq;
    int sigma;
    read_config_cen(zq, sigma, "/home/evan/code/radar-localization/config/cen_2018.txt");
    cen_2018_features(rd1.fft_data.clone(), zq, sigma, 58, rd1.targets);
    cen_2018_features(rd2.fft_data.clone(), zq, sigma, 58, rd2.targets);
    targets_to_point_cloud(rd2, source_cloud);
    targets_to_point_cloud(rd1, target_cloud);
    cv::Mat result = targets_to_cartesian_points(rd1, 800, 800, 0.2);
    Eigen::Matrix4f pose = pcl_icp_registration(source_cloud, target_cloud, 30);
    std::cout << pose.block<3, 1>(0, 3)[0] << " " << pose.block<3, 1>(0, 3)[1] << " " 
        << pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0)[0] << std::endl;
    cv::imshow("",result);
    cv::waitKey(0);
}

int main()
{
    // test();
    // test_features_registration("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result", 
    //     kstrongest, icp);
    // test_features_registration("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result", 
    //     cen2018, icp);
    find_best_config_for_cen_2018();
    return 0;
}