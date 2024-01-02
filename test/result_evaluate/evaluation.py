import numpy as np
import math

def get_transform(x, y, theta):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    T = [[cos_theta, sin_theta, x], 
         [-sin_theta, cos_theta, y], 
         [0, 0, 1]]
    return T

def translation_error(T):
    return calculate_distance(T[0, 2], T[1, 2])

def calculate_distance(x, y):
    return math.sqrt(x * x + y * y)

def trajectory_distance(gt_pose, length):
    acc_length = []
    gt_pose_list = []

    str_ind = 0
    cur_len = 0

    for ind in range(0, len(gt_pose) - 1):
        if ind == str_ind:
            cur_len = 0
        cur_len = cur_len + calculate_distance(gt_pose[ind][1], gt_pose[ind][2])
        if cur_len >= length:
            acc_length.append(cur_len)
            gt_pose_list.append([str_ind, ind])
            str_ind = ind + 1
    return gt_pose_list, acc_length

def calculate_ave_error(seq_err):
    seq_len = len(seq_err)
    err = 0
    for one_err in seq_err:
        err = err + one_err

    return err / seq_len

def calculate_seq_err(gt_pose, result_pose, length):
    seq_err = []
    gt_pose_list, acc_length = trajectory_distance(gt_pose, length)
    for t_ind in range(0, len(gt_pose_list) - 1):
        one_trajectory = gt_pose_list[t_ind]
        T_gt = np.eye(3, 3)
        T_result = np.eye(3, 3)
        for ind in range(one_trajectory[0], one_trajectory[1]):
            T_gt_ = get_transform(gt_pose[ind][1], gt_pose[ind][2], gt_pose[ind][3])
            T_result_ = get_transform(result_pose[ind][1], result_pose[ind][2], result_pose[ind][3])
            T_gt = np.matmul(T_gt, T_gt_)
            T_result = np.matmul(T_result, T_result_)
        one_err = translation_error(np.matmul(np.linalg.inv(T_result), T_gt))
        seq_err.append(one_err / acc_length[t_ind])
    ave_error = calculate_ave_error(seq_err) * 100
    print(ave_error)
    return ave_error
