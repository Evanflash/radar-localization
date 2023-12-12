import math
import numpy as np
import evaluation as evo

if __name__ == "__main__":
    data = np.loadtxt(
        open("/home/evan/extra/datasets/20190110-114621/accuracy.csv"),
        delimiter=",",
        skiprows=1)
    result = [[1547120787893007, 0, 0, 0]]
    result_m = [[1547120787893007, 0, 0, 0]]
    result_dm = [[1547120787893007, 0, 0, 0]]
    gt_pose = [[1547120787893007, 0, 0, 0]]
    for row in data:
        gt_pose.append([row[7], row[3], row[4], row[5]])
        result.append([row[7], row[0], row[1], row[2]])
        result_m.append([row[7], row[8], row[9], row[10]])
        result_dm.append([row[7], row[11], row[12], row[13]])
    print("ransac")
    evo.calculate_seq_err(gt_pose, result, 100)
    print("ransac with motion distrotion")
    evo.calculate_seq_err(gt_pose, result_m, 100)
    print("ransac with motion distrotion and doppler")
    evo.calculate_seq_err(gt_pose, result_dm, 100)