# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 04:24:26 2022

@author: siddg
"""

import numpy as np
import cv2
import time
from scipy.io import loadmat
from scipy import interpolate
import random
import matplotlib.pyplot as plt
from point2planeICP import icp

pi = np.pi
cos = np.cos
sin = np.sin

def plotTrajectory(x, y, scan):

    plt.figure()

    origin = [-3.55, -0.997, 0]

    mapData = plt.imread("savedMap.pgm")

    Ny, Nx = np.shape(mapData)

    res = 0.01
    
    a = np.arcsin(2*pose[0, 3]*pose[0, 6])
    R = np.array([
        [cos(a), -sin(a)],
        [sin(a), cos(a)]
        ])
    
    scan = R @ scan + pose[0, :2].reshape(2, 1)

    plt.imshow(mapData, extent=[origin[0], origin[0] + Nx*res, origin[1], origin[1] + Ny*res], cmap  = 'Greys_r')

    plt.plot(x, y, lw = 2.5)

    plt.plot(scan[0], scan[1])

    plt.plot(pose[:, 0], pose[:, 1], lw = 1.00)

    plt.show()

def getPointCloud():

    data = loadmat("f1tenthROSData4.mat")

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

pts, pose, t_scan, t_odom = getPointCloud()

points = np.load("interpolatedPoints.npy")

normals = np.load("interpolatedNormals.npy")

#plotTrajectory(pose[:, 0], pose[:, 1], pts[0].T)

q_w = pose[:, 3]
q_z = pose[:, 6]

sin_pose = 2*q_w*q_z
cos_pose = q_w*q_w - q_z*q_z

N2 = len(points)

T_pose = np.eye(3)

theta0 = np.arcsin(sin_pose[0])

T_pose[:2, :2] = np.array([[cos(theta0), -sin(theta0)], [sin(theta0), cos(theta0)]])
T_pose[:2, 2:] = np.array([pose[0][0], pose[0][1]]).reshape(2, 1)

T_pose[0, 2] = pose[0][0]
T_pose[1, 2] = pose[0][1]

source = points[250]
target = points[200]
target_normal = normals[200]
T = icp(source, target, target_normal, max_iterations=50)
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


