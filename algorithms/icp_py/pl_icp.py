# import cv2
import numpy as np
# import copy
# import pylab
# import time
# import sys
import sklearn.neighbors
import scipy.optimize

R_90 = np.array([
	[0, 1],
	[-1, 0],
])


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


def loss(x, p, q, n):
	T = pose2tf(x)
	num = np.shape(p)[1]
	p_1 = np.ones([3, num])
	p_1[:2] = p
	delta = (T @ p_1)[:2] - q
	dot = np.einsum('ij,ij->i', n.T, delta.T)

	return np.linalg.norm(dot) ** 2


def icp(src, dst, max_iterations=4):
	p = np.array(src, copy=True).astype(np.float32)
	q = np.array(dst, copy=True).astype(np.float32)
	Tr = np.eye(3)
	for i in range(max_iterations):
		distances, indices = sklearn.neighbors.NearestNeighbors(n_neighbors=2, algorithm='auto', p=3).fit(
			q.T).kneighbors(p.T)
		t = q[:, indices[:, 0]] - q[:, indices[:, 1]]
		n = (R_90 @ t)
		n = n / np.linalg.norm(n, axis=0)
		x = scipy.optimize.minimize(loss, [0, 0, 0], args=(p, q[:, indices[:, 0]], n)).x
		T = pose2tf(x)
		p = T[:2, :2] @ p + T[:2, 2:].reshape(2, 1)
		# p = cv2.transform(np.array([p.T], copy=True).astype(np.float32), T[:2])[0].T
		Tr = T @ Tr
	return Tr
