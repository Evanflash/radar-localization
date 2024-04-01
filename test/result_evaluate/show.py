import numpy as np
import evaluate_utils
import matplotlib.pyplot as plt

lengths = [100, 200, 300, 400, 500, 600, 700, 800]

def trajectoryDistances(poses):
    dist = [0]
    for i in range(1, len(poses)):
        P1 = poses[i - 1]
        P2 = poses[i]
        dx = P1[0, 2] - P2[0, 2]
        dy = P1[1, 2] - P2[1, 2]
        dist.append(dist[i-1] + np.sqrt(dx**2 + dy**2))
    return dist

def lastFrameFromSegmentLength(dist, first_frame, length):
    for i in range(first_frame, len(dist)):
        if dist[i] > dist[first_frame] + length:
            return i
    return -1

def get_inverse_tf(T):
    T2 = np.identity(3)
    R = T[0:2, 0:2]
    t = T[0:2, 2]
    t = np.reshape(t, (2, 1))
    T2[0:2, 0:2] = R.transpose()
    t = np.matmul(-1 * R.transpose(), t)
    T2[0, 2] = t[0]
    T2[1, 2] = t[1]
    return T2

def enforce_orthogonality(R):
    epsilon = 0.001
    if abs(R[0, 0] - R[1, 1]) > epsilon or abs(R[1, 0] + R[0, 1]) > epsilon:
        print("ERROR: this is not a proper rigid transformation!")
    a = (R[0, 0] + R[1, 1]) / 2
    b = (-R[1, 0] + R[0, 1]) / 2
    sum = np.sqrt(a**2 + b**2)
    a /= sum
    b /= sum
    R[0, 0] = a; R[0, 1] = b
    R[1, 0] = -b; R[1, 1] = a

def get_transform(x, y, theta):
    R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    if np.linalg.det(R) != 1.0:
        enforce_orthogonality(R)
    T = np.identity(3)
    T[0:2, 0:2] = R
    T[0, 2] = x
    T[1, 2] = y
    return T

def rotationError(pose_error):
    return abs(np.arcsin(pose_error[0, 1]))

def translationError(pose_error):
    return np.sqrt(pose_error[0, 2]**2 + pose_error[1, 2]**2)

def calcSequenceErrors(poses_gt, poses_res):
    err = []
    step_size = 4 # Every second
    # Pre-compute distances from ground truth as reference
    dist = trajectoryDistances(poses_gt)
    # print(dist[-1])

    for first_frame in range(0, len(poses_gt), step_size):
        for i in range(0, len(lengths)):
            length = lengths[i]
            last_frame = lastFrameFromSegmentLength(dist, first_frame, length)
            if last_frame == -1:
                continue
            # Compute rotational and translation errors
            pose_delta_gt = np.matmul(get_inverse_tf(poses_gt[first_frame]), poses_gt[last_frame])
            pose_delta_res = np.matmul(get_inverse_tf(poses_res[first_frame]), poses_res[last_frame])
            pose_error = np.matmul(get_inverse_tf(pose_delta_res), pose_delta_gt)
            r_err = rotationError(pose_error)
            t_err = translationError(pose_error)
            # Approx speed
            num_frames = float(last_frame - first_frame + 1)
            speed = float(length) / (0.25 * num_frames)
            err.append([first_frame, r_err/float(length), t_err/float(length), length, speed])
    return err

def calcAbsoluteTrajectoryError(poses_gt, poses_res):
    error = 0
    for T_gt, T_res in zip(poses_gt, poses_res):
        T_err = np.matmul(get_inverse_tf(T_res), T_gt)
        t_err = T_err[0:2, 2].reshape(2, 1)
        error += (np.linalg.norm(t_err) ** 2)
    error /= len(poses_gt)
    return np.sqrt(error)

def getStats(err):
    t_err = 0
    r_err = 0
    for e in err:
        t_err += e[2]
        r_err += e[1]
    t_err /= float(len(err))
    r_err /= float(len(err))
    return t_err, r_err

def compute_route(result_name):
    result = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + result_name + ".txt")
    err = []
    ate = []
    poses_gt = []
    poses_res = []
    for ind in range(0, len(result)):
        T_gt_ = get_transform(result[ind][4], result[ind][5], result[ind][6])
        T_res_ = get_transform(result[ind][1], result[ind][2], result[ind][3])

        poses_gt.append(T_gt_)
        poses_res.append(T_res_)
    rpe_err = 0
    rpe_num = 0
    for first_frame in range(0, len(poses_gt) - 1):
        pose_delta_gt = np.matmul(get_inverse_tf(poses_gt[first_frame]), poses_gt[first_frame - 1])
        pose_delta_res = np.matmul(get_inverse_tf(poses_res[first_frame]), poses_res[first_frame - 1])
        pose_error = np.matmul(get_inverse_tf(pose_delta_res), pose_delta_gt)
        t_err = translationError(pose_error)
        rpe_err = rpe_err + t_err
        rpe_num = rpe_num + 1

    err.extend(calcSequenceErrors(poses_gt, poses_res))
    ate.append(calcAbsoluteTrajectoryError(poses_gt, poses_res))

    ate = np.array(ate)

    t_err, r_err = getStats(err)
    return (t_err * 100), (r_err * 100 * 180 / np.pi), (rpe_err / rpe_num), (np.mean(ate))


if __name__ == "__main__":
    # result_name_list = ["contral/large_0.5", "contral/large_1", "contral/large_1.5", "contral/large_2",
    #                     "contral/large_2.5", "contral/large_3", "contral/large_3.5", "contral/large_4",
    #                     "contral/large_4.5", "contral/large_5", "contral/large_5.5", "contral/large_6",
    #                     "contral/large_6.5", "contral/large_7", "contral/large_7.5", "contral/large_8",
    #                     "contral/large_8.5", "contral/large_9", "contral/large_9.5", "contral/large_10",
    #                     "contral/large_10.5", "contral/large_11", "contral/large_11.5", "contral/large_12"]
    result_name_list = ["rsize/large_0.5", "rsize/large_1", "rsize/large_1.5", "rsize/large_2",
                    "rsize/large_2.5", "rsize/large_3", "rsize/large_3.5", "rsize/large_4",
                    "rsize/large_4.5", "rsize/large_5", "rsize/large_5.5", "rsize/large_6",
                    "rsize/large_6.5", "rsize/large_7", "rsize/large_7.5", "rsize/large_8"]
    result_terr_list = []
    result_rerr_list = []
    result_trpe_list = []
    result_tate_list = []

    for name in result_name_list:
        terr, rerr, trpe, tate = compute_route(name)
        result_terr_list.append(terr)
        result_rerr_list.append(rerr)
        result_trpe_list.append(trpe)
        result_tate_list.append(tate)

    # x = [0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12]
    x = [0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8]
    fig = plt.figure(0)
    plt.plot(x, result_terr_list)
    fig = plt.figure(1)
    plt.plot(x, result_rerr_list)
    fig = plt.figure(2)
    plt.plot(x, result_trpe_list)
    fig = plt.figure(4)
    plt.plot(x, result_tate_list)

    plt.show()

    
    