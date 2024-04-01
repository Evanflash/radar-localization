import evaluate_utils
import matplotlib.pyplot as plt
import evaluation as evo


if __name__ == '__main__':
    first_file_name = "0110/try"
    # second_file_name = "large/large_mdad_thres_mul"
    # big_data = False
    # timestamps = evaluate_utils.read_timestamps("/home/evan/extra/datasets/large/radar_change.timestamps")
    # gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/large/gt/radar_odometry_change.csv")
    # gt_pose1 = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/20190110-114621/gt/radar_odometry_change.csv")
    result1 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + first_file_name + ".txt")
    # result2 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + second_file_name + ".txt")

    # evaluate_utils.compare_two_result(gt_pose, result1, result2)
    # for i in range(0, len(result1)):
    #     result2[i][2] = result1[i][2]    

    # if len(result2) < len(result1):
    #     for ind in range(len(result2), len(result1) - 1):
    #         result2.append(result1[ind])

    # for i in range(0, len(result1) - 1):
    #     result1[i][3] = -result1[i][3]

    # evo.calculate_seq_err(gt_pose1, result2, 100)
    # print(evaluate_utils.gt_long(gt_pose1))


    route_my1 = []
    route_gt = []
    for i in range(0, len(result1)):
        route_my1.append([result1[i][1], result1[i][2]])
        route_gt.append([result1[i][4], result1[i][5]])



    # route_gt = evaluate_utils.calculate_final_pose(gt_pose)
    # route_my1 = evaluate_utils.calculate_final_pose(result1)
    # route_my2 = evaluate_utils.calculate_final_pose(result2)
    route = [route_gt, route_my1]
    evaluate_utils.show_route(route)

    # 矩阵相乘计算误差
    # if big_data == True:
    #     ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result1)
    #     ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result2)
    #     drift1 = evaluate_utils.calculate_drift(gt_pose1, result1)
    #     drift2 = evaluate_utils.calculate_drift(gt_pose1, result2)

    # else:
    #     ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error_by_matmul(gt_pose, result1)
    #     ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error_by_matmul(gt_pose, result2)
    #     drift1 = evaluate_utils.calculate_drift(gt_pose, result1)
    #     drift2 = evaluate_utils.calculate_drift(gt_pose, result2)
    
    # print("------------------" + first_file_name + "--------------------------")
    # print(str(evaluate_utils.get_average(ex1)) + " " + 
    #     str(evaluate_utils.get_average(ey1)) + " " + str(evaluate_utils.get_average(exy21)) + 
    #     " " + str(evaluate_utils.get_average(eyaw1)) + " " + str(drift1))
    # evaluate_utils.calculate_percentage(exy21, 0.1)
    # evaluate_utils.calculate_percentage(exy21, 0.2)
    # evaluate_utils.calculate_percentage(exy21, 0.3)
    # evaluate_utils.calculate_percentage(exy21, 0.4)
    # evaluate_utils.calculate_percentage(exy21, 0.5)
    # evaluate_utils.calculate_percentage(exy21, 0.6)
    # evaluate_utils.calculate_percentage(exy21, 0.7)
    # evaluate_utils.calculate_percentage(exy21, 0.8)
    # evaluate_utils.calculate_percentage(exy21, 0.9)
    # evaluate_utils.calculate_percentage(exy21, 1)
    # print("------------------" + second_file_name + "--------------------------")
    # print(str(evaluate_utils.get_average(ex2)) + " " + 
    #     str(evaluate_utils.get_average(ey2)) + " " + str(evaluate_utils.get_average(exy22)) + 
    #     " " + str(evaluate_utils.get_average(eyaw2)) + " " + str(drift2))
    # evaluate_utils.calculate_percentage(exy22, 0.1)
    # evaluate_utils.calculate_percentage(exy22, 0.2)
    # evaluate_utils.calculate_percentage(exy22, 0.3)
    # evaluate_utils.calculate_percentage(exy22, 0.4)
    # evaluate_utils.calculate_percentage(exy22, 0.5)
    # evaluate_utils.calculate_percentage(exy22, 0.6)
    # evaluate_utils.calculate_percentage(exy22, 0.7)
    # evaluate_utils.calculate_percentage(exy22, 0.8)
    # evaluate_utils.calculate_percentage(exy22, 0.9)
    # evaluate_utils.calculate_percentage(exy22, 1)
    # evaluate_utils.print_result_compare(ex1, first_file_name, ex2, second_file_name, 1, "x")
    # evaluate_utils.print_result_compare(ey1, first_file_name, ey2, second_file_name, 2, "y")
    # evaluate_utils.print_result_compare(eyaw1, first_file_name, eyaw2, second_file_name, 3, "yaw")
    # evaluate_utils.print_result_compare(exy21, first_file_name, exy22, second_file_name, 4, "xy")
    # plt.show()

# def align(gt, result):
#     timestamps = []
#     start = result[0][0]
#     end = result[len(result) - 1][0]
#     for one in gt:
#         if one[0] >= start and one[0] <= end:
#             timestamps.append(one[0])
#     return timestamps

# def get_absolute_poses(timestamps, poses, relative):
#     absolute_poses = []

#     if relative == 1:
#         T = np.identity(3)
#         theta = 0
#         for pose in poses:
#             T_ = get_transform(pose[1], pose[2], pose[3])
#             T = np.matmul(T, T_)
#             theta = theta + pose[2]
#             if theta >= 2 * np.pi:
#                 theta = theta - 2 * np.pi
#             if theta <= -2 * np.pi:
#                 theta = theta + 2 * np.pi
#             absolute_pose = [pose[0], T[0, 2], T[1, 2], theta]
#             absolute_poses.append(absolute_pose)
#     else:
#         absolute_poses = poses
    
#     result = []
#     cur = 0
#     for timestamp in timestamps:
#         for ind in range(cur, len(absolute_poses) - 1):
#             if not (absolute_poses[ind][0] <= timestamp and absolute_poses[ind + 1][0] >= timestamp):
#                 continue
#             l1 = timestamp - absolute_poses[ind][0]
#             l2 = absolute_poses[ind + 1][0] - timestamp
#             w1 = l2 / (l1 + l2)
#             w2 = l1 / (l1 + l2)
#             x = w1 * absolute_poses[ind][1] + w2 * absolute_poses[ind + 1][1]
#             y = w1 * absolute_poses[ind][2] + w2 * absolute_poses[ind + 1][2]
#             yaw = w1 * absolute_poses[ind][3] + w2 * absolute_poses[ind + 1][3]
#             result.append([timestamp, x, y, yaw])
#             cur = ind - 1
#             break

#     T_base = get_transform(result[0][1], result[0][2], result[0][3])
#     T_base = np.linalg.inv(T_base)

#     transformation = []
#     for ind in range(0, len(result)):
#         T = get_transform(result[ind][1], result[ind][2], result[ind][3])
        
#         T = np.matmul(T_base, T)
#         R = T[0:2, 0:2]

#         if np.linalg.det(R) != 1.0:
#             enforce_orthogonality(R)
#             T[0:2, 0:2] = R
#         transformation.append(T)
#     return transformation