import evaluate_utils
import matplotlib.pyplot as plt
import evaluation as evo


if __name__ == '__main__':
    first_file_name = "my_registration_1228_2_2"
    second_file_name = "imu"
    big_data = True
    timestamps = evaluate_utils.read_timestamps("/home/evan/extra/datasets/large/radar.timestamps")
    gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/large/gt/radar_odometry_change.csv")
    gt_pose1 = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/20190110-114621/gt/radar_odometry_change.csv")
    result1 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + first_file_name + ".txt")
    result2 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + second_file_name + ".txt")
    
    # if len(result2) < len(result1):
    #     for ind in range(len(result2), len(result1) - 1):
    #         result2.append(result1[ind])

    # for i in range(0, len(result1) - 1):
    #     result1[i][3] = gt_pose1[i][3]

    # evo.calculate_seq_err(gt_pose1, result2, 100)
    # print(evaluate_utils.gt_long(gt_pose1))

    route_gt = evaluate_utils.calculate_final_pose(gt_pose1)
    route_my1 = evaluate_utils.calculate_final_pose(result1)
    route_my2 = evaluate_utils.calculate_final_pose(result2)
    route = [route_gt, route_my1, route_my2]
    evaluate_utils.show_route(route)

    # 矩阵相乘计算误差
    if big_data == True:
        ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result1)
        ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result2)
        drift1 = evaluate_utils.calculate_drift(gt_pose1, result1)
        drift2 = evaluate_utils.calculate_drift(gt_pose1, result2)

    else:
        ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error_by_matmul(gt_pose, result1)
        ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error_by_matmul(gt_pose, result2)
        drift1 = evaluate_utils.calculate_drift(gt_pose, result1)
        drift2 = evaluate_utils.calculate_drift(gt_pose, result2)
    
    print("------------------" + first_file_name + "--------------------------")
    print(str(evaluate_utils.get_average(ex1)) + " " + 
        str(evaluate_utils.get_average(ey1)) + " " + str(evaluate_utils.get_average(exy21)) + 
        " " + str(evaluate_utils.get_average(eyaw1)) + " " + str(drift1))
    evaluate_utils.calculate_percentage(exy21, 0.1)
    evaluate_utils.calculate_percentage(exy21, 0.2)
    evaluate_utils.calculate_percentage(exy21, 0.3)
    evaluate_utils.calculate_percentage(exy21, 0.4)
    evaluate_utils.calculate_percentage(exy21, 0.5)
    evaluate_utils.calculate_percentage(exy21, 0.6)
    evaluate_utils.calculate_percentage(exy21, 0.7)
    evaluate_utils.calculate_percentage(exy21, 0.8)
    evaluate_utils.calculate_percentage(exy21, 0.9)
    evaluate_utils.calculate_percentage(exy21, 1)
    print("------------------" + second_file_name + "--------------------------")
    print(str(evaluate_utils.get_average(ex2)) + " " + 
        str(evaluate_utils.get_average(ey2)) + " " + str(evaluate_utils.get_average(exy22)) + 
        " " + str(evaluate_utils.get_average(eyaw2)) + " " + str(drift2))
    evaluate_utils.calculate_percentage(exy22, 0.1)
    evaluate_utils.calculate_percentage(exy22, 0.2)
    evaluate_utils.calculate_percentage(exy22, 0.3)
    evaluate_utils.calculate_percentage(exy22, 0.4)
    evaluate_utils.calculate_percentage(exy22, 0.5)
    evaluate_utils.calculate_percentage(exy22, 0.6)
    evaluate_utils.calculate_percentage(exy22, 0.7)
    evaluate_utils.calculate_percentage(exy22, 0.8)
    evaluate_utils.calculate_percentage(exy22, 0.9)
    evaluate_utils.calculate_percentage(exy22, 1)
    # evaluate_utils.print_result_compare(ex1, first_file_name, ex2, second_file_name, 1, "x")
    # evaluate_utils.print_result_compare(ey1, first_file_name, ey2, second_file_name, 2, "y")
    # evaluate_utils.print_result_compare(eyaw1, first_file_name, eyaw2, second_file_name, 3, "yaw")
    # evaluate_utils.print_result_compare(exy21, first_file_name, exy22, second_file_name, 4, "xy")
    # plt.show()

