#include "features.hpp"
#include "radar_utils.hpp"
#include <opencv2/opencv.hpp>

void compare_two_filter(radar_data first_rd, radar_data second_rd)
{
    int rows = 800;
    int cols = 800;
    k_strongest_features(first_rd.fft_data, 58, 12, first_rd.targets);
    cen_2018_features(second_rd.fft_data, 3, 17, 58, second_rd.targets);
    cv::Mat first = targets_to_cartesian_points(first_rd, rows, cols, 0.2);
    cv::Mat second = targets_to_cartesian_points(second_rd, rows, cols, 0.2);
    cv::Mat image(rows, 2 * cols, CV_8U);
    first.copyTo(image.colRange(0, cols));
    second.copyTo(image.colRange(cols, 2 * cols));
    cv::imshow("image", image);
    cv::waitKey(0);
}

void test_features_registration(const std::string timestamp_file_path, const std::string data_file_path,
                const std::string save_file_path, features_model fm, registration_model rm)
{   
    using ll = long long;
    std::string output_file_pat = save_file_path + "/" + 
        features_model_string[fm] + "_" + registration_model_string[rm] + ".txt";
    std::vector<ll> radar_timestamp = read_timestamp_file(timestamp_file_path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(uint i = 0; i < radar_timestamp.size(); ++i){
        radar_data rd;
        radar_data_split(data_file_path, std::to_string(radar_timestamp[i]), rd);
        if(fm == kstrongest){
            k_strongest_features(rd.fft_data, 58, 12, rd.targets);
        } else if(fm == cen2018){
            cen_2018_features(rd.fft_data, 4, 21, 58, rd.targets);
        }

        if(i <= 0){
            targets_to_point_cloud(rd, target_cloud);
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        targets_to_point_cloud(rd, source_cloud);

        Eigen::Matrix4f pose;
        if(rm == icp){
            pose = pcl_icp_registration(source_cloud, target_cloud, 30);
        }
        save_transformation(output_file_pat, pose, radar_timestamp[i]);

        target_cloud = source_cloud;
    }
}

void find_best_config_for_cen_2018()
{
    using ll = long long;
    std::vector<v_pose> gt_pose = read_gt_pose("/home/evan/extra/datasets/tiny/gt/radar_odometry.csv");
    std::vector<ll> radar_timestamp = read_timestamp_file("/home/evan/extra/datasets/tiny/radar.txt");
    std::string data_file_path = "/home/evan/extra/datasets/tiny/radar";
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    std::fstream log_output("/home/evan/code/radar-localization/log/find_best_cen_config.txt", std::ios::out);

    float min_error = 10000000000;
    int best_zq = 1;
    int best_sigma = 1;
    for(int zq = 1; zq < 10; ++zq){
        for(int sigma = 1; sigma < 50; sigma = sigma + 2){
            float error = 0;
            for(uint i = 0; i < radar_timestamp.size(); ++i){
                radar_data rd;
                radar_data_split(data_file_path, std::to_string(radar_timestamp[i]), rd);
                cen_2018_features(rd.fft_data, zq, sigma, 58, rd.targets);

                if(i <= 0){
                    targets_to_point_cloud(rd, target_cloud);
                    continue;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                targets_to_point_cloud(rd, source_cloud);

                Eigen::Matrix4f pose;
                pose = pcl_icp_registration(source_cloud, target_cloud, 30);

                v_pose gt = find_cloest_pose(gt_pose, radar_timestamp[i]);

                float dx = gt.x - pose.block<3, 1>(0, 3)[0];
                float dy = gt.y - pose.block<3, 1>(0, 3)[1];

                error += std::sqrt(dx * dx + dy * dy);
                // std::cout << gt.timestamp << std::endl;
                target_cloud = source_cloud;
            }
            if(error < min_error){
                best_zq = zq;
                best_sigma = sigma;
            }
            std::cout << "cur zq = " << zq << "," << "cur sigma = " << sigma << ","
                << "cur error = " << error << std::endl;
            log_output << "cur zq = " << zq << "," << "cur sigma = " << sigma << ","
                << "cur error = " << error << std::endl;
        }
    }
    std::cout << best_zq << " " << best_sigma << std::endl;
    std::fstream output("/home/evan/code/radar-localization/config/cen_2018.txt", std::ios::out);
    output << best_zq << std::endl << best_sigma << std::endl;
    output.close();
    log_output.close();
}

void find_best_kstrongest_config()
{
    
}