import numpy as np
import cv2
import time
from scipy.io import loadmat
from scipy import interpolate
import random
import matplotlib.pyplot as plt
from icp import icp

pi = np.pi
cos = np.cos
sin = np.sin

def plotTrajectory(x, y, scan):

    plt.figure()

    origin = [-3.55, -0.997, 0]

    mapData = plt.imread("savedMap.pgm")

    Ny, Nx = np.shape(mapData)

    res = 0.01

    plt.imshow(mapData, extent=[origin[0], origin[0] + Nx*res, origin[1], origin[1] + Ny*res], cmap  = 'Greys_r')

    plt.plot(x, y, lw = 2.5)

    #plt.plot(scan[0], scan[1])

    plt.plot(pose[:, 0], pose[:, 1], lw = 1.00)

    plt.show()

def getPointCloud():

    data = loadmat("f1tenthROSData5.mat")

    r = data['laser_scans']

    pose = data['odom_pose']

    t_odom = data['t_odom']

    t_scan = data['t_scan']

    angle = np.linspace(-3*pi/4, 3*pi/4, 1081)

    N = len(r)

    origin = [-3.55, -0.997, 0]

    points = {}

    for k in range(N):
        r_filt = r[k][np.where(r[k] < 50)]
        angle_filt = angle[np.where(r[k] < 50)]
        points[k] = np.vstack((r_filt*cos(angle_filt + 0*pi/2) + 0*pose[0][0], r_filt*sin(angle_filt + 0*pi/2) + 0*pose[0][1])).T

    return points, pose, t_scan, t_odom

points, pose, t_scan, t_odom = getPointCloud()

#plotTrajectory(pose[:, 0], pose[:, 1], points[0].T)

Flag = False

if Flag:
    q_w = pose[:, 3]
    q_z = pose[:, 6]

    sin_pose = 2*q_w*q_z
    cos_pose = q_w*q_w - q_z*q_z

    N2 = len(points)

    T_pose = np.zeros([(N2), 3, 3])

    theta0 = np.arcsin(sin_pose[0])

    T_pose[0][:2, :2] = np.array([[cos(theta0), -sin(theta0)], [sin(theta0), cos(theta0)]])
    T_pose[0][:2, 2:] = np.array([pose[0][0], pose[0][1]]).reshape(2, 1)

    T_pose[0][0, 2] = pose[0][0]
    T_pose[0][1, 2] = pose[0][1]

    T_pose_all = np.zeros([20, N2, 3, 3])

    T_rel = np.zeros([N2, 3, 3])

    T_rel[0] = np.eye(3)

    # T_pose[0, 1, 2] = 0.3

    #t_start = time.time();
    t_steps = np.zeros(20)
    for j in range(20):
        t_start = time.time()
        for i in range(N2-1):
            #print(i)
            source = points[i+1].T
            target = points[i].T
            T_update = icp(source, target, max_iterations=j+2)
            #T_rel[i+1] = T_update
            T_pose[i+1] = T_pose[i] @ T_update
        t_end = time.time()
        t_steps[j] = (t_end - t_start)/len(points)
        T_pose_all[j] = T_pose
        print(j, t_steps[j])
       
    np.save('poseDataAll(2).npy', T_pose_all)
    #np.save('timeData(2).npy', T_pose_all)

    pose_time = np.arange(len(sin_pose))/len(sin_pose)

    icp_time = np.arange(N2)/N2



    #np.save('/home/schmidd/Documents/Python/UPenn_Research/poseTest.npy', T_rel)

    result = cv2.transform(np.array([source.T], copy=True).astype(np.float32), T_pose[0] @ T_rel[0]).T

    tx = T_pose[:, 0, 2]
    ty = T_pose[:, 1, 2]

    plotTrajectory(tx, ty, result)

else:
   
    q_w = pose[:, 3]
    q_z = pose[:, 6]

    sin_pose = 2*q_w*q_z
    cos_pose = q_w*q_w - q_z*q_z
   
    theta0 = np.arcsin(sin_pose[0])
   
    T_pose = np.zeros([(1), 3, 3])

    T_pose[0][:2, :2] = np.array([[cos(theta0), -sin(theta0)], [sin(theta0), cos(theta0)]])
    T_pose[0][:2, 2:] = np.array([pose[0][0], pose[0][1]]).reshape(2, 1)

    T_pose[0][0, 2] = pose[0][0]
    T_pose[0][1, 2] = pose[0][1]
    i = 50
    source = points[10].T
    target = points[0].T
    T = icp(source, target, max_iterations=50)
    result = cv2.transform(np.array([source.T], copy=True).astype(np.float32), T_pose[0] @ T).T
    target_tf = cv2.transform(np.array([target.T], copy=True).astype(np.float32), T_pose[0]).T
    source_tf = cv2.transform(np.array([source.T], copy=True).astype(np.float32), T_pose[0]).T

    plt.figure()

    origin = [-3.55, -0.997, 0]

    mapData = plt.imread("savedMap.pgm")

    Ny, Nx = np.shape(mapData)

    res = 0.01

    plt.imshow(mapData, extent=[origin[0], origin[0] + Nx*res, origin[1], origin[1] + Ny*res], cmap  = 'Greys_r')

    plt.plot(target_tf[0], target_tf[1], lw = 2.5, label = 'target')
    plt.plot(source_tf[0], source_tf[1], lw = 1.25, label = 'source')
    plt.plot(result[0], result[1], lw = 2.5, label = 'result', linestyle = '--')

    plt.grid()

    plt.axis('equal')

    plt.show()

pose_time = np.arange(len(sin_pose))/len(sin_pose)

icp_time = np.arange(N2)/(N2-1)*np.max(pose_time)*0.999999

f = interpolate.interp1d(pose_time, pose.T)

pose_interp = f(icp_time)

error = np.zeros(20)

pose_icp = np.zeros([20, 4, len(icp_time)])
pose_pf = np.zeros([20, 4, len(icp_time)])


for i in range(20):
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
   
np.save("errorData(2).npy", error)
np.save("timeData(2).npy", t_steps)
np.save("icpPose(2).npy", pose_icp)
np.save("pfPose(2).npy", pose_pf)

# T, T_list, error = icp(source, target, max_iterations=25)

# init_pose = np.array([2, 3, np.pi/6])

# T =         np.array([[np.cos(init_pose[2]),-np.sin(init_pose[2]),init_pose[0]],
#                       [np.sin(init_pose[2]), np.cos(init_pose[2]),init_pose[1]],
#                       [0,                    0,                   1          ]])

# result = cv2.transform(np.array([target.T], copy=True).astype(np.float32), T).T

# T, T_list, error = icp(source, target, max_iterations=25)

# dx = T[0,2]
# dy = T[1,2]
# rotation = np.arcsin(T[0,1]) * 360 / 2 / np.pi

# print("T",T)
# print("error",error)
# print("rotationÂ°",rotation)
# print("dx",dx)
# print("dy",dy)

# #result = cv2.transform(np.array([source.T], copy=True).astype(np.float32), T).T

# plt.figure()

# plt.plot(target[0], target[1], lw = 2.5, label = 'target')
# plt.plot(source[0], source[1], lw = 2.5, label = 'source')
# plt.plot(result[0], result[1], lw = 2.5, label = 'result', linestyle = '--')

# plt.grid()

# plt.axis('equal')

# plt.figure()

# plt.plot(error, lw = 2.5, marker = 'o')

# plt.grid()

# plt.show()