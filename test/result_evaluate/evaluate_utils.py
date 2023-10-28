import csv
import math
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
            result.append([int(l[0]), float(l[1]), float(l[2]), float(l[3])])
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