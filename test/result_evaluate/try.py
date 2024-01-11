import math
import numpy as np
import evaluation as evo
import evaluate_utils
import matplotlib.pyplot as plt
    

if __name__ == "__main__":
    data = np.loadtxt(
        open("/home/evan/code/radar-localization/test/result/my_registration_try.txt"),
        str,
        delimiter=" ",
        skiprows=1)
    result_raw = []
    for row in data:
        result_raw.append([float(row[0]), float(row[1]), -float(row[2]), float(row[3])])

    
    result_w = []
    cur_x = 0
    cur_y = 0
    cur_yaw = 0
    for p in result_raw:
        cur_x = cur_x + p[2] * 0.02
        cur_y = cur_y + p[1] * 0.02
        cur_yaw = p[3] - np.pi / 2
        result_w.append([p[0], cur_x, cur_y, cur_yaw])

    gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/large/gt/radar_odometry_change.csv")
    
    result_final_w = []
    cur = 0
    for pose in gt_pose:
        for ind in range(cur, len(result_w) - 1):
            if not (result_w[ind][0] <= pose[0] and result_w[ind + 1][0] >= pose[0]):
                continue
            l1 = pose[0] - result_w[ind][0]
            l2 = result_w[ind + 1][0] - pose[0]
            w1 = l2 / (l1 + l2)
            w2 = l1 / (l1 + l2)
            x = w1 * result_w[ind][1] + w2 * result_w[ind + 1][1]
            y = w1 * result_w[ind][2] + w2 * result_w[ind + 1][2]
            yaw = w1 * result_w[ind][3] + w2 * result_w[ind + 1][3]
            result_final_w.append([pose[0], x, y, yaw])
            cur = ind
            break

    result = []
    
    for ind in range(1, len(result_final_w)):
        t = evaluate_utils.get_transform(result_final_w[ind - 1][1],
                                     result_final_w[ind - 1][2],
                                     result_final_w[ind - 1][3])
        T = np.matmul(np.linalg.inv(t), evaluate_utils.get_transform(
            result_final_w[ind][1],
            result_final_w[ind][2],
            result_final_w[ind][3]))
        x = T[0][2]
        y = T[1][2]
        yaw = np.arcsin(T[0][1])
        result.append([result_final_w[ind][0], x, y, yaw])
    result.insert(0, gt_pose[0])

    route_gt = evaluate_utils.calculate_final_pose(gt_pose)
    route_ins = evaluate_utils.calculate_final_pose(result)
    route = [route_gt, route_ins]
    evaluate_utils.show_route(route)

    # f = open("/home/evan/code/radar-localization/test/result/ins.txt", "w")
    # for line in result:
    #     f.write(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + str(line[3]) + '\n')