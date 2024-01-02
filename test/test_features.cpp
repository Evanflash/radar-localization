#include "features.hpp"
#include "radar_utils.hpp"
#include "filter.hpp"
#include "registration.hpp"
#include <opencv2/opencv.hpp>

void compare_two_filter(radar_data first_rd, radar_data second_rd)
{
    int rows = 800;
    int cols = 800;
    k_strongest_features(first_rd.fft_data, 58, 12, first_rd.targets);
    cen_2018_features(second_rd.fft_data, 1, 9, 58, second_rd.targets);
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
        features_model_string[fm] + "_" + registration_model_string[rm] + "_big_data.txt";
    std::vector<ll> radar_timestamp = read_timestamp_file(timestamp_file_path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(uint i = 0; i < radar_timestamp.size(); ++i){
        radar_data rd;
        radar_data_split(data_file_path, std::to_string(radar_timestamp[i]), rd);
        if(fm == kstrongest){
            k_strongest_features(rd.fft_data, 58, 12, rd.targets);
        } else if(fm == cen2018){
            cen_2018_features(rd.fft_data, 1, 9, 58, rd.targets);
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

pcl::PointCloud<pcl::PointXYZI>::Ptr transform_frame(
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Vector3d pose)
{
    Eigen::Matrix3d T;
    double cos_theta = cos(pose[2]);
    double sin_theta = sin(pose[2]);
    T << cos_theta, sin_theta, pose[0],
        -sin_theta, cos_theta, pose[1],
        0, 0, 1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    result_cloud -> reserve(point_cloud -> size());

    for(auto point : point_cloud -> points){
        Eigen::Vector3d p(point.x, point.y, 1);
        p = T * p;
        result_cloud -> push_back(pcl::PointXYZI(p[0], p[1], 0, point.intensity));
    }

    return result_cloud;
}

std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> get_target_clouds
    (std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, std::queue<Eigen::Vector3d> poses)
{
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> result;
    Eigen::Vector3d tmp_poses = poses.front();
    while(!clouds.empty()){
        result.push(transform_frame(clouds.front(), poses.front() - tmp_poses));
        clouds.pop();
        poses.pop();
    }
    return result;
}

void test_my_registration(const std::string timestamp_file_path, const std::string data_file_path,
                const std::string save_file_path)
{
    using ll = long long;
    std::string output_file_pat = save_file_path + "/my_registration_try_0.txt";
    std::fstream output(output_file_pat.c_str(), std::ios::out);

    Eigen::Vector3d last_pose(0, 2.4, 0);
    double filter_grid_size = 0.5;

    std::vector<ll> radar_timestamp = read_timestamp_file(timestamp_file_path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(uint i = 0; i < radar_timestamp.size(); ++i){
        radar_data rd;
        radar_data_split(data_file_path, std::to_string(radar_timestamp[i]), rd);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_value = 
        //     k_strongest_filter(rd, 12, 0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_value = 
            k_strongest_filter(rd, 12, 0, last_pose);
            
        if(i <= 0){
            // target_cloud = extract_surf_point(k_strongest_value, 5, 2, 1);
            target_cloud = k_strongest_value;
            // target_cloud = extract_flat_surf_points(k_strongest_value, filter_grid_size);
            continue;
        }
        // pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud =
        //     extract_surf_point(k_strongest_value, 5, 2, 1);

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = k_strongest_value;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = 
        //     extract_flat_surf_points(k_strongest_value, filter_grid_size);

        Eigen::Vector3d pose = 
            point_to_line_registration_weighted(source_cloud, target_cloud, last_pose);

        output << rd.timestamp << " " << pose[1] << " " << 
            pose[0] << " " << pose[2] << std::endl;

        // target_cloud = source_cloud;

        k_strongest_value = k_strongest_filter(rd, 12, 0, pose);
        target_cloud = k_strongest_value;
        // target_cloud = extract_flat_surf_points(k_strongest_value, filter_grid_size);
        last_pose = pose;
    }
    output.close();
}

void test_my_registration_scan_to_mulkeyframes(const std::string timestamp_file_path, const std::string data_file_path,
                const std::string save_file_path)
{
    using ll = long long;
    std::string output_file_pat = save_file_path + "/my_registration_try_3_1.5.txt";
    std::fstream output(output_file_pat.c_str(), std::ios::out);

    Eigen::Vector3d last_pose(0, 0, 0);
    Eigen::Vector3d last_relative_pose(0, 2.4, 0);
    Eigen::Vector3d last_keyframe_pose(0, 0, 0);

    int keyframe_nums = 3;
    double keyframe_min_dis = 1.5;
    double keyframe_min_the = 0.09;

    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> target_clouds;
    std::queue<Eigen::Vector3d> target_poses;

    std::vector<ll> radar_timestamp = read_timestamp_file(timestamp_file_path);

    for(uint i = 0; i < radar_timestamp.size(); ++i){
        radar_data rd;
        radar_data_split(data_file_path, std::to_string(radar_timestamp[i]), rd);
        pcl::PointCloud<pcl::PointXYZI>::Ptr k_strongest_value = 
            k_strongest_filter(rd, 12, 0, last_relative_pose);
            
        if(i <= 0){
            target_clouds.push(k_strongest_value);
            target_poses.push(Eigen::Vector3d(0, 0, 0));
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = k_strongest_value;

        // 组装目标帧
        std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> tmp = get_target_clouds(target_clouds, target_poses);

        Eigen::Vector3d pose = point_to_line_registration_weighted_mulkeyframe_cauchy(
            source_cloud, tmp, last_pose + last_relative_pose - target_poses.front());
        // Eigen::Vector3d tmp = 
        //     point_to_line_registration_weighted(
        //         source_cloud, target_clouds.front(), last_pose + last_relative_pose) - last_pose;

        // std::cout << tmp[0] << " " << tmp[1] << std::endl;

        last_relative_pose = pose + target_poses.front() - last_pose;
        last_pose = pose + target_poses.front();

        output << rd.timestamp << " " << last_relative_pose[1] << " " << 
            last_relative_pose[0] << " " << last_relative_pose[2] << std::endl;

        if(sqrt((last_pose[0] - last_keyframe_pose[0]) * (last_pose[0] - last_keyframe_pose[0]) + 
            (last_pose[1] - last_keyframe_pose[1]) * (last_pose[1] - last_keyframe_pose[1])) >= keyframe_min_dis ||
            abs(last_pose[2] - last_keyframe_pose[2]) >= keyframe_min_the)
        {
            k_strongest_value = k_strongest_filter(rd, 12, 0, last_relative_pose);
            target_clouds.push(k_strongest_value);
            target_poses.push(last_pose);
            if((int)target_clouds.size() > keyframe_nums){
               target_clouds.pop();
               target_poses.pop(); 
            }
            last_keyframe_pose = last_pose;
        }
    }
    output.close();
}

void find_best_config_for_cen_2018()
{
    using ll = long long;
    std::vector<v_pose> gt_pose = read_gt_pose("/home/evan/extra/datasets/tiny/gt/radar_odometry.csv");
    std::vector<ll> radar_timestamp = read_timestamp_file("/home/evan/extra/datasets/tiny/radar.txt");
    std::string data_file_path = "/home/evan/extra/datasets/tiny/radar";
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    std::fstream log_output("/home/evan/code/radar-localization/log/find_best_cen_k_config.txt", std::ios::out);

    double min_error = 10000000000;
    int best_zq = 1;
    int best_sigma = 1;
    for(int zq = 1; zq < 10; ++zq){
        for(int sigma = 1; sigma < 50; sigma = sigma + 2){
            double error = 0;
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

                double dx = gt.x - pose.block<3, 1>(0, 3)[0];
                double dy = gt.y - pose.block<3, 1>(0, 3)[1];

                error += std::sqrt(dx * dx + dy * dy);
                // std::cout << gt.timestamp << std::endl;
                target_cloud -> clear();
                *target_cloud += *source_cloud;
            }
            if(error < min_error){
                best_zq = zq;
                best_sigma = sigma;
                min_error = error;
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
