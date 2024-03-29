import csv
import math
import numpy as np
import matplotlib.pyplot as plt

def read_timestamps(file_path):
    timestamps = []
    with open(file_path, 'r') as f:
        data = f.readlines()
        for line in data:
            value = line.split()
            timestamps.append(int(value[0]))
    return timestamps

def read_gt_pose(file_path):
    gt_pose = []
    with open(file_path) as gt_file:
        data = csv.reader(gt_file)
        k = True
        for line in data:
            if k == True:
                k = False
                continue
            one_pose = [int(line[8]), float(line[2]), float(line[3]), float(line[7])]
            gt_pose.append(one_pose)

    return gt_pose

def find_gt_pose(gt_pose, timestamp):
    index = 0
    min_value = abs(timestamp - gt_pose[0][0])
    cur_ind = 0
    for one_pose in  gt_pose:
        if abs(timestamp - one_pose[0]) < min_value:
            index = cur_ind
            min_value = abs(timestamp - one_pose[0])
        cur_ind = cur_ind + 1
    
    return gt_pose[index][0], gt_pose[index][1], gt_pose[index][2], gt_pose[index][3]

def read_result(result_file):
    result = []
    with open(result_file) as f:
        data = f.readlines()
        for line in data:
            l = line.split()
            result.append([int(float(l[0])), float(l[1]), float(l[2]), float(l[3])])
    return result

def calculate_error(gt_pose, result):
    ex = []
    ey = []
    exy2 = []
    eyaw = []
    for pose in result:
        t, x, y, yaw = find_gt_pose(gt_pose, pose[0])
        dx = abs(x - pose[1])
        dy = abs(y - pose[2])
        dxy2 = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(yaw - pose[3])
        ex.append(dx)
        ey.append(dy)
        exy2.append(dxy2)
        eyaw.append(dyaw)
    return ex, ey, exy2, eyaw

def print_result_compare(result1, name1, result2, name2, fig, title):
    num1 = len(result1) + 1
    num2 = len(result2) + 1
    plt.figure(fig)
    plt.plot(list(range(1, num1)), result1, label = name1)
    plt.plot(list(range(1, num2)), result2, label = name2)
    plt.title(title)
    plt.legend()

def get_average(e):
    l = len(e)
    all_e = 0
    for ei in e:
        all_e = all_e + ei
    return all_e / l

def calculate_percentage(e, s):
    num = 0
    for ei in e:
        if ei <= s:
            num = num + 1
    
    result = (num / len(e)) * 100
    print("the percentage below " + str(s) + " is " + str(result) + "%")


def calculate_final_pose(data):
    result = []
    result.append([0, 0])
    T = np.eye(3, 3)
    for one_data in  data:
        T_t = get_transform(one_data[1], one_data[2], one_data[3])
        T = np.matmul(T, T_t)
        result.append([T[0][2], T[1][2]])
    return result

def show_route(route):
    plt.figure(0)
    for ind in range(0, len(route)):
        one_route = list(map(list, zip(*route[ind])))
        plt.plot(one_route[0], one_route[1], label = "第" + str(ind) + "条路径")
    plt.show()


def get_transform(x, y, yaw):
    cos_theta = math.cos(yaw)
    sin_theta = math.sin(yaw)
    T = [[cos_theta, sin_theta, x], 
        [-sin_theta, cos_theta, y], 
        [0, 0, 1]]
    T = np.array(T)
    return T

def calculate_error_by_matmul(gt_pose, result):
    ex = []
    ey = []
    exy2 = []
    eyaw = []
    
    for pose in result:
        t, x, y, yaw = find_gt_pose(gt_pose, pose[0])
        T_ = get_transform(pose[1], pose[2], pose[3])
        T = get_transform(x, y, yaw)
        T_error = np.matmul(np.linalg.inv(T_), T)
        ex.append(np.abs(T_error[0, 2]))
        ey.append(np.abs(T_error[1, 2]))
        exy2.append(math.sqrt(T_error[0, 2] * T_error[0, 2] + T_error[1, 2] * T_error[1, 2]))
        eyaw.append(math.acos(T_error[0, 0]))
    return ex, ey, exy2, eyaw

def calculate_drift(gt_pose, result):
    T_gt_all = np.eye(3, 3)
    T_result_all = np.eye(3, 3)
    for pose in result:
        t, x, y, yaw = find_gt_pose(gt_pose, pose[0])
        T_ = get_transform(pose[1], pose[2], pose[3])
        T = get_transform(x, y, yaw)
        T_gt_all = np.matmul(T_gt_all, T_)
        T_result_all = np.matmul(T_result_all, T)
    T_error = np.matmul(np.linalg.inv(T_result_all), T_gt_all)
    return np.sqrt(T_error[0, 2] * T_error[0, 2] + T_error[1, 2] * T_error[1, 2])

def gt_long(gt_pose):
    result = 0
    for pose in gt_pose:
        result = result + math.sqrt(pose[1] * pose[1] + pose[2] * pose[2])

    return result

def error_vs_speed(gt_pose, result, speed):
    e = []
    for v in speed:
        exy2 = []
        for pose in result:
            t, x, y, yaw = find_gt_pose(gt_pose, pose[0])
            v_cur = np.sqrt(x * x + y * y) * 4
            if v_cur < v:
                T_ = get_transform(pose[1], pose[2], pose[3])
                T = get_transform(x, y, yaw)
                T_error = np.matmul(np.linalg.inv(T_), T)
                exy2.append(math.sqrt(T_error[0, 2] * T_error[0, 2] + T_error[1, 2] * T_error[1, 2]))
        e.append(get_average(exy2))
    return e

def compare_two_result(gt_pose, result1, result2):
    speed = range(1, 13)
    e1 = error_vs_speed(gt_pose, result1, speed)
    e2 = error_vs_speed(gt_pose, result2, speed)
    plt.plot(speed, e1)
    plt.plot(speed, e2)
    plt.show()