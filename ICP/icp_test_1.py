import numpy as np
import cv2
import time
import yaml
from scipy.io import loadmat
from scipy import interpolate
import random
import matplotlib.pyplot as plt
from icp import icp

pi = np.pi
cos = np.cos
sin = np.sin

with open('map4.yaml') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
    map_name = data['image']
    res = data['resolution']
    origin = data['origin']

def plotTrajectory(x, y, scan):

    plt.figure()

    mapData = plt.imread(map_name)

    Ny, Nx = np.shape(mapData)

    plt.imshow(mapData, extent=[origin[0], origin[0] + Nx*res, origin[1], origin[1] + Ny*res], cmap  = 'Greys_r')

    plt.plot(x, y, lw = 2.5)

    plt.plot(scan[0], scan[1], lw = 2.5)

    plt.show()

def getPointCloud():

    data = loadmat("f1tenthROSData6.mat")

    r = data['laser_scans']

    pose = data['odom_pose']

    t_odom = data['t_odom']

    t_scan = data['t_scan']

    angle = np.linspace(-3*pi/4, 3*pi/4, 1081)

    N = len(r)

    points = {}

    for k in range(N):
        r_filt = r[k][np.where(r[k] < 50)]
        angle_filt = angle[np.where(r[k] < 50)]
        points[k] = np.vstack((r_filt*cos(angle_filt), r_filt*sin(angle_filt)))

    return points, pose, t_scan, t_odom

points, pose, t_scan, t_odom = getPointCloud()

q_w = pose[:, 3]
q_z = pose[:, 6]

sin_pose = 2*q_w*q_z
cos_pose = q_w*q_w - q_z*q_z

N_scans = 1700

T_pose = np.zeros([(N_scans), 3, 3])

theta0 = np.arcsin(sin_pose[0])

T_pose[0][:2, :2] = np.array([[cos(theta0), -sin(theta0)], [sin(theta0), cos(theta0)]])
T_pose[0][:2, 2:] = np.array([pose[0][0], pose[0][1]]).reshape(2, 1)

T_pose[0][0, 2] = pose[0][0]
T_pose[0][1, 2] = pose[0][1]

N_steps = 1

T_pose_all = np.zeros([N_steps, N_scans, 3, 3])

t_steps = np.zeros(N_steps)
for j in range(N_steps):
    t_start = time.time()
    for i in range(N_scans-1):
        print(i)
        source = points[i+1]
        target = points[i]
        T_update = icp(source, target, max_iterations=50)
        T_pose[i+1] = T_pose[i] @ T_update
    t_end = time.time()
    t_steps[j] = (t_end - t_start)/len(points)
    T_pose_all[j] = T_pose
    print(j, t_steps[j])

pose_time = np.arange(len(sin_pose))/len(sin_pose)

icp_time = np.arange(N_scans)/(N_scans-1)*np.max(pose_time)*0.999999

f = interpolate.interp1d(pose_time, pose.T)

pose_interp = f(icp_time)

error = np.zeros(N_steps)

pose_icp = np.zeros([N_steps, 4, len(icp_time)])
pose_pf = np.zeros([N_steps, 4, len(icp_time)])


for i in range(N_steps):
    tx = T_pose_all[i, :, 0, 2]
    ty = T_pose_all[i, :, 1, 2]
    si = T_pose_all[i, :, 1, 0]
    ci = T_pose_all[i, :, 0, 0]

    pose_icp[i] = np.vstack((tx, ty, si, ci))

    px = pose_interp[0]
    py = pose_interp[1]
    cp = pose_interp[3]**2 - pose_interp[6]**2
    sp = 2*pose_interp[3]*pose_interp[6]

    pose_pf[i] = np.vstack((px, py, sp, cp))

    error[i] = np.linalg.norm(pose_pf[i] - pose_icp[i])
    
np.save('poseDataAll(3).npy', T_pose_all)    
np.save("errorData(3).npy", error)
np.save("timeData(3).npy", t_steps)
np.save("icpPose(3).npy", pose_icp)
np.save("pfPose(3).npy", pose_pf)

# src = np.array([points[0].T], copy=True).astype(np.float32)

# src_tf = cv2.transform(src, T_pose[0][:2])[0].T

plotTrajectory(pose_icp[0][0, :], pose_icp[0][1, :], [0, 0])