import numpy as np
# import cv2
import time
from scipy.io import loadmat
from scipy import interpolate
# import random
import matplotlib.pyplot as plt
from pl_icp import icp
import yaml

pi = np.pi
cos = np.cos
sin = np.sin

# with open('homeMap.yaml') as f:
#     data = yaml.load(f, Loader=yaml.FullLoader)
#     map_name = data['image']
#     res = data['resolution']
#     origin = data['origin']

origin = [-3.55, -0.997, 0]

mapData = plt.imread('savedMap.pgm')

res = 1e-2


def pose2tf(x):
    T = np.eye(3)
    R = np.array([
        [np.cos(x[2]), -np.sin(x[2])],
        [np.sin(x[2]), np.cos(x[2])]
    ])
    t = x[:2].reshape(2, 1)

    T[:2, :2] = R
    T[:2, 2:] = t

    return T


def plotTrajectory(x, y, scan):
    plt.figure()

    # mapData = plt.imread(map_name)

    Ny, Nx = np.shape(mapData)

    plt.imshow(mapData, extent=[origin[0], origin[0] + Nx * res, origin[1], origin[1] + Ny * res], cmap='Greys_r')

    plt.plot(x, y, lw=2.5)

    plt.plot(scan[0], scan[1], lw=2.5)

    plt.show()


def getPointCloud():
    data = loadmat('f1tenthROSData4.mat')

    r = data['laser_scans']

    pose = data['odom_pose']

    t_odom = data['t_odom']

    t_scan = data['t_scan']

    angle = np.linspace(-3 * pi / 4, 3 * pi / 4, 1081)

    N = len(r)

    points = {}

    for k in range(N):
        r_filt = r[k][np.where(r[k] < 50)]
        angle_filt = angle[np.where(r[k] < 50)]
        points[k] = np.vstack((r_filt * cos(angle_filt), r_filt * sin(angle_filt)))

    return points, pose, t_scan, t_odom


points, pose, t_scan, t_odom = getPointCloud()

p = points[0]

x0 = np.zeros(3)

x0[:2] = pose[0, :2] + 0 * np.array([-0.7, 0.02])

x0[2] = 0 * np.arctan2(pose[0, 6], pose[0, 3]) + 90 * pi / 180

T0 = pose2tf(x0)

p_tf = T0[:2, :2] @ p + T0[:2, 2:].reshape(2, 1)
# p_tf = cv2.transform(np.array([p.T], copy=True).astype(np.float32), T0[:2])[0].T

plotTrajectory(pose[:, 0], pose[:, 1], p_tf)

N2 = len(points)

T_pose = np.zeros([(N2), 3, 3])

T_pose[0] = T0

# T_pose[0][:2, :2] = np.array([[cos(x0[2]), -sin(x0[2])], [sin(x0[2]), cos(x0[2])]])
# T_pose[0][:2, 2:] = x0[:2].reshape(2, 1)

# T_pose[0][0, 2] = pose[0][0]
# T_pose[0][1, 2] = pose[0][1]

J = 1

T_pose_all = np.zeros([J, N2, 3, 3])

t_steps = np.zeros(J)

for j in range(J):
    t_start = time.time()
    for i in range(N2 - 1):
        source = points[i + 1]
        target = points[i]
        T_update = icp(source, target, max_iterations=25)
        T_pose[i + 1] = T_pose[i] @ T_update
    t_end = time.time()
    t_steps[j] = (t_end - t_start) / len(points)
    T_pose_all[j] = T_pose
    print(j, t_steps[j])

tx = T_pose[:, 0, 2]
ty = T_pose[:, 1, 2]

plotTrajectory(tx, ty, pose[:, :2].T)

q_w = pose[:, 3]
q_z = pose[:, 6]

sin_pose = 2 * q_w * q_z
cos_pose = q_w * q_w - q_z * q_z

pose_time = np.arange(len(sin_pose)) / len(sin_pose)

icp_time = np.arange(N2) / N2

pose_time = np.arange(len(sin_pose)) / len(sin_pose)

icp_time = np.arange(N2) / (N2 - 1) * np.max(pose_time) * 0.999999

f = interpolate.interp1d(pose_time, pose.T)

pose_interp = f(icp_time)

error = np.zeros(J)

pose_icp = np.zeros([J, 4, len(icp_time)])
pose_pf = np.zeros([J, 4, len(icp_time)])

for i in range(J):
    tx = T_pose_all[i, :, 0, 2]
    ty = T_pose_all[i, :, 1, 2]
    si = T_pose_all[i, :, 1, 0]
    ci = T_pose_all[i, :, 0, 0]

    pose_icp[i] = np.vstack((tx, ty, si, ci))

    px = pose_interp[0]
    py = pose_interp[1]
    cp = pose_interp[3] ** 2 - pose_interp[6] ** 2
    sp = 2 * pose_interp[3] * pose_interp[6]

    pose_pf[i] = np.vstack((px, py, sp, cp))

    error[i] = np.linalg.norm(pose_pf[i] - pose_icp[i])

np.save('poseDataAll(2).npy', T_pose_all)
np.save("errorData(2).npy", error)
np.save("timeData(2).npy", t_steps)
np.save("icpPose(2).npy", pose_icp)
np.save("pfPose(2).npy", pose_pf)
