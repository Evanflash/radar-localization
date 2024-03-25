import numpy as np
import evaluate_utils

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
    print(dist[-1])

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


if __name__ == "__main__":
    result_name = "normal_md_doppler_mdad/1_2_10/ceres_registration_md_other"
    gt_name = 0
    keyframe = 1
    if gt_name:
        gt_name = "20190110-114621"
    else:
        gt_name = "large"

    timestamps = evaluate_utils.read_timestamps("/home/evan/extra/datasets/" + gt_name + "/radar_change.timestamps")
    gt_pose = evaluate_utils.read_gt_pose("/home/evan/extra/datasets/" + gt_name + "/gt/radar_odometry_change.csv")
    result = evaluate_utils.read_result("/home/evan/code/radar-localization/test/result/" + result_name + ".txt")

    err = []
    ate = []

    T_gt = np.identity(3)
    T_res = np.identity(3)

    poses_gt = []
    poses_res = []

    timestamps.remove(timestamps[len(timestamps) - 1])

    if keyframe:
        # 所有帧都保存
        for ind in range(0, len(result)):
            T_gt_ = get_transform(gt_pose[ind][1], gt_pose[ind][2], gt_pose[ind][3])
            T_res_ = get_transform(result[ind][1], result[ind][2], result[ind][3])

            T_gt = np.matmul(T_gt, T_gt_)
            T_res = np.matmul(T_res, T_res_)

            R_gt = T_gt[0:2, 0:2]
            R_res = T_res[0:2, 0:2]

            if np.linalg.det(R_gt) != 1.0:
                enforce_orthogonality(R_gt)
                T_gt[0:2, 0:2] = R_gt
            if np.linalg.det(R_res) != 1.0:
                enforce_orthogonality(R_res)
                T_res[0:2, 0:2] = R_res

            poses_gt.append(T_gt)
            poses_res.append(T_res)
    else:
        # 仅存关键帧
        result_after = []
        cur = 0
        for timestamp in timestamps:
            for ind in range(cur, len(result) - 1):
                if not (result[ind][0] <= timestamp and result[ind + 1][0] >= timestamp):
                    continue
                l1 = timestamp - result[ind][0]
                l2 = result[ind + 1][0] - timestamp
                w1 = l2 / (l1 + l2)
                w2 = l1 / (l1 + l2)
                x = w1 * result[ind][1] + w2 * result[ind + 1][1]
                y = w1 * result[ind][2] + w2 * result[ind + 1][2]
                yaw = w1 * result[ind][3] + w2 * result[ind + 1][3]
                result_after.append([timestamp, x, y, yaw])
                cur = ind - 1
                break
        
        result = result_after

        for ind in range(0, len(result)):
            T_res = get_transform(result[ind][1], result[ind][2], result[ind][3])
            R_res = T_res[0:2, 0:2]
            if np.linalg.det(R_res) != 1.0:
                enforce_orthogonality(R_res)
                T_res[0:2, 0:2] = R_res
            poses_res.append(T_res)

        gt_pose = gt_pose[0 : len(result) - 1]

        poses_gt.insert(0, poses_res[0])
        for i in range(0, len(gt_pose)):
            T_gt_ = get_transform(gt_pose[i][1], gt_pose[i][2], gt_pose[i][3])
            T_gt = np.matmul(T_gt, T_gt_)
            R_gt = T_gt[0:2, 0:2]
            if np.linalg.det(R_gt) != 1.0:
                enforce_orthogonality(R_gt)
                T_gt[0:2, 0:2] = R_gt
            poses_gt.append(T_gt)
    
    err.extend(calcSequenceErrors(poses_gt, poses_res))
    ate.append(calcAbsoluteTrajectoryError(poses_gt, poses_res))

    ate = np.array(ate)

    t_err, r_err = getStats(err)
    print(result_name)
    print('t_err: {} %'.format(t_err * 100))
    print('r_err: {} deg/m'.format(r_err * 180 / np.pi))
    print('ATE: {} m'.format(np.mean(ate)))