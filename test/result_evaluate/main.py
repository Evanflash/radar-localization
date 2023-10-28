import evaluate_utils
import matplotlib.pyplot as plt

if __name__ == '__main__':
    timestamps = evaluate_utils.read_timestamps("/home/evan/extra/datasets/tiny/radar.txt")
    gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/tiny/gt/radar_odometry.csv")
    result1 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/kstrongest_icp.txt")
    result2 = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/cen2018_icp.txt")
    ex1, ey1, exy21, eyaw1 = evaluate_utils.calculate_error(gt_pose, result1)
    ex2, ey2, exy22, eyaw2 = evaluate_utils.calculate_error(gt_pose, result2)
    evaluate_utils.print_result_compare(ex1, "kstrongest", ex2, "cen2018", 1, "x")
    evaluate_utils.print_result_compare(ey1, "kstrongest", ey2, "cen2018", 2, "y")
    evaluate_utils.print_result_compare(exy21, "kstrongest", exy22, "cen2018", 3, "xy")
    plt.show()

