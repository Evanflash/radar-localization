import evaluate_utils
import matplotlib.pyplot as plt
import evaluation as evo

if __name__ == '__main__':
    timestamps = evaluate_utils.read_timestamps("/home/evan/extra/datasets/tiny/radar.txt")
    gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/tiny/gt/radar_odometry.csv")
    gt_pose1 = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/20190110-114621/gt/radar_odometry.csv")
    result1 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/my_registration_big_data_true.txt")
    result2 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/my_registration_big_data_dis5.txt")
    
    evo.calculate_seq_err(gt_pose1, result1, 400)
    # print(evaluate_utils.gt_long(gt_pose1))

    # route_gt = evaluate_utils.calculate_final_pose(gt_pose1)
    # route_my = evaluate_utils.calculate_final_pose(result1)
    
    # evaluate_utils.show_route(route_gt, route_my)

    # ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error(gt_pose1, result1)
    # ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error(gt_pose1, result2)
    # 矩阵相乘计算误差
    ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result1)
    ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error_by_matmul(gt_pose1, result2)
    
    print("------------------kstrongest--------------------------")
    print(str(evaluate_utils.get_average(ex1)) + " " + 
        str(evaluate_utils.get_average(ey1)) + " " + str(evaluate_utils.get_average(exy21)))
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
    print("------------------my_registration--------------------------")
    print(str(evaluate_utils.get_average(ex2)) + " " + 
        str(evaluate_utils.get_average(ey2)) + " " + str(evaluate_utils.get_average(exy22)) + 
        " " + str(evaluate_utils.get_average(eyaw2)))
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
    evaluate_utils.print_result_compare(ex1, "kstrongest", ex2, "my_registration_last_pose", 1, "x")
    evaluate_utils.print_result_compare(ey1, "kstrongest", ey2, "my_registration_last_pose", 2, "y")
    evaluate_utils.print_result_compare(exy21, "kstrongest", exy22, "my_registration_last_pose", 3, "xy")
    plt.show()
    # import numpy as np
    # file_name = "/home/evan/code/radar-localization/test/result/my_registration_big_data_true.txt"
    # a = np.loadtxt("/home/evan/code/radar-localization/test/result/my_registration_big_data.txt", delimiter= ' ')
    # b = a
    # for i in range(0, len(b) - 1):
    #     b[i][3] = -b[i][3]
    # np.savetxt(file_name, b, fmt = "%s", delimiter=' ')

