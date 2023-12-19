#include <iostream>
#include <fstream>

#include "filter.hpp"
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

void compare_cen_kstrongest()
{
    radar_data rd_cen;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131050856549", rd_cen);
    radar_data rd_kstrongest;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131050856549", rd_kstrongest);
    compare_two_filter(rd_cen, rd_kstrongest);
}

void test(){
    radar_data rd1;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131050856549", rd1);
    k_strongest_features(rd1.fft_data.clone(), 58, 2, rd1.targets);
    radar_data rd2;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131051108813", rd2);
    k_strongest_features(rd2.fft_data.clone(), 58, 2, rd2.targets);
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
    // cen_2018_features(rd1.fft_data.clone(), zq, sigma, 58, rd1.targets);
    // cen_2018_features(rd2.fft_data.clone(), zq, sigma, 58, rd2.targets);
    // k_strongest_features(rd1.fft_data.clone(), 58, 12, rd1.targets);
    // k_strongest_features(rd2.fft_data.clone(), 58, 12, rd2.targets);
    // my_features(rd1.fft_data, rd1.targets);
    // my_features(rd2.fft_data, rd2.targets);
    targets_to_point_cloud(rd2, source_cloud);
    targets_to_point_cloud(rd1, target_cloud);
    cv::Mat result = targets_to_cartesian_points(rd1, 800, 800, 0.2);
    cv::Mat result1 = targets_to_cartesian_points(rd2, 800, 800, 0.2);
    cv::Mat image(800, 1600, CV_8U);
    result.copyTo(image.colRange(0, 800));
    result1.copyTo(image.colRange(800, 1600));
    cv::imshow("", image);
    cv::waitKey(0);
    Eigen::Matrix4f pose = pcl_icp_registration(source_cloud, target_cloud, 30);
    std::cout << pose.block<3, 1>(0, 3)[0] << " " << pose.block<3, 1>(0, 3)[1] << " " 
        << pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0)[0] << std::endl;
    
}

void test1()
{
    radar_data rd1;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131050856549", rd1);
    radar_data rd2;
    radar_data_split("/home/evan/code/radar-localization/test", "1547131051108813", rd2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = k_strongest_filter(rd1, 12, 0, Eigen::Vector3f(0.006803, 0.864929, 0.001772));
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = k_strongest_filter(rd2, 12, 0, Eigen::Vector3f(0.006803, 0.864929, 0.001772));

    float grid_size = 0.5;
    // target_cloud = extract_flat_surf_points(target_cloud, grid_size);
    // source_cloud = extract_flat_surf_points(source_cloud, grid_size);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr target_tmp_cloud = k_strongest_filter(rd1, 5, 0);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr source_tmp_cloud = k_strongest_filter(rd2, 5, 0);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = extract_surf_point(target_tmp_cloud, 5, 2, 1);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = extract_surf_point(source_tmp_cloud, 5, 2, 1);

    cv::Mat target_image = pointcloud_to_cartesian_points(target_cloud, 800, 800, 0.2);
    cv::Mat source_image = pointcloud_to_cartesian_points(source_cloud, 800, 800, 0.2);

    cv::Mat image(800, 1600, CV_8U);
    target_image.copyTo(image.colRange(0, 800));
    source_image.copyTo(image.colRange(800, 1600));

    Eigen::Matrix4f pose = pcl_icp_registration(source_cloud, target_cloud, 30);
    std::cout << "pcl_icp_result" << std::endl;
    std::cout << pose.block<3, 1>(0, 3)[0] << " " << pose.block<3, 1>(0, 3)[1] << " " 
        << pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0)[0] << std::endl;

    Eigen::Vector3f point_to_line_result = 
        point_to_line_registration_weighted(source_cloud, target_cloud, Eigen::Vector3f(0.006803, 0.864929, 0.001772));
    std::cout << "point_to_line_result" << std::endl;
    std::cout << point_to_line_result[0] << " " << point_to_line_result[1] << " "
        << point_to_line_result[2] << std::endl;

    cv::imshow("", image);
    cv::waitKey(0);
}

int main()
{
    // test1();
    // radar_data rd1;
    // radar_data_split("/home/evan/code/radar-localization/test", "1547131050856549", rd1);
    // cen_2018_features(rd1.fft_data.clone(), 1, 9, 58, rd1.targets);
    // compare_cen_kstrongest();
    // test();
    // test_features_registration("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result", 
    //     kstrongest, icp);
    // test_features_registration("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result", 
    //     cen2018, icp);
    
    // find_best_config_for_cen_2018();
    // my_features(rd1.fft_data);
    // test_my_registration("/home/evan/extra/datasets/20190110-114621/radar_change.timestamps", 
    //     "/home/evan/extra/datasets/20190110-114621/radar", 
    //     "/home/evan/code/radar-localization/test/result");
    // test_my_registration("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result");
    test_my_registration_scan_to_mulkeyframes("/home/evan/extra/datasets/20190110-114621/radar_change.timestamps", 
        "/home/evan/extra/datasets/20190110-114621/radar", 
        "/home/evan/code/radar-localization/test/result");
    // test_my_registration_scan_to_mulkeyframes("/home/evan/extra/datasets/tiny/radar.txt", 
    //     "/home/evan/extra/datasets/tiny/radar", 
    //     "/home/evan/code/radar-localization/test/result");
    // test_features_registration("/home/evan/extra/datasets/20190110-114621/radar.timestamps", 
    // "/home/evan/extra/datasets/20190110-114621/radar", 
    // "/home/evan/code/radar-localization/test/result", 
    // kstrongest, icp);
    return 0;
}