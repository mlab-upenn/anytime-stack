# import cv2
import numpy as np
# import copy
# import pylab
# import time
# import sys
import sklearn.neighbors
import scipy.optimize

pi = np.pi
cos = np.cos
sin = np.sin


def pose2tf(xi):
	T = np.array([
		[cos(xi[0]), -sin(xi[0]), xi[1]],
		[sin(xi[0]), cos(xi[0]), xi[2]],
		[0, 0, 1]
	])
	return T


def cost(xi, p, q, nq):
	T = pose2tf(xi)
	R = T[:2, :2]
	t = T[:2, 2:]
	p_tf = R @ p + t
	J = 0.5 * np.sum(np.square(np.einsum('ij,ij->i', nq.T, (p_tf - q).T)))

	return J


def jacobian(xi, p, q, nq):
	xi_shift = np.copy(xi)
	xi_shift[0] += pi / 2
	T = pose2tf(xi)
	R = T[:2, :2]
	t = T[:2, 2:].reshape(2, 1)
	R1 = pose2tf(xi_shift)[:2, :2]
	p_tf = R @ p + t
	r = np.einsum('ij,ij->i', nq.T, (p_tf - q).T)
	n = len(r)

	grad = np.zeros([3, n])
	grad[0] = np.einsum('ij,ij->i', nq.T, (R1 @ p).T) * r
	grad[1:] = r * nq
	grad_sum = np.sum(grad, axis=1)

	return grad_sum


def hessian(xi, p, q, nq):
	xi_shift = np.copy(xi)
	T = pose2tf(xi)
	R = T[:2, :2]
	t = T[:2, 2:].reshape(2, 1)
	xi_shift[0] += pi / 2
	R1 = pose2tf(xi_shift)[:2, :2]
	xi_shift[0] += pi / 2
	R2 = pose2tf(xi_shift)[:2, :2]
	p_tf = R @ p + t
	r = np.einsum('ij,ij->i', nq.T, (p_tf - q).T)
	n = len(r)

	grad = np.zeros([3, n])
	grad[0] = np.einsum('ij,ij->i', nq.T, (R1 @ p).T) * r
	grad[1:] = r * nq

	H1 = np.zeros([n, 3, 3])
	H1[:, 0, 0] = np.einsum('ij,ij->i', nq.T, (R2 @ p).T) * r

	H2 = grad @ grad.T

	H = H1 + H2

	H_sum = np.sum(H, axis=0)

	return H_sum


def icp(source, target, target_normals, max_iterations=4):
	# xi_1 = scipy.optimize.minimize(fun = cost, x0 = xi, args = (p, q, nq), method = 'BFGS').x
	# xi_2 = scipy.optimize.minimize(fun = cost, x0 = xi, args = (p, q, nq), method = 'BFGS', jac=jacobian).x
	# xi_3 = scipy.optimize.minimize(fun = cost, x0 = xi, args = (p, q, nq), method = 'dogleg', jac = jacobian, hess = hessian).x
	xi_0 = np.zeros(3)
	p = np.array(source, copy=True).astype(float)
	q = np.array(target, copy=True).astype(float)
	nq = np.array(target_normals, copy=True).astype(float)
	Tr = pose2tf(xi_0)
	xi = np.copy(xi_0)
	for i in range(max_iterations):
		d, indices = sklearn.neighbors.NearestNeighbors(n_neighbors=1, algorithm='auto', p=3).fit(q).kneighbors(p)
		idx = indices.flatten()
		print(np.shape(p[:, idx]))
		xi_rel = scipy.optimize.minimize(
			fun=cost,
			x0=xi_0,
			args=(p[:, idx], q[:, idx], nq[:, idx]),
			method='BFGS',
			jac=jacobian,
		).x
		T = pose2tf(xi_rel)
		xi[1:] = (xi[1:] * np.matrix(T[:2, :2]).T).A
		xi += xi_rel
		p = T[:2, :2] @ p + T[:2, 2:].reshape(2, 1)
		# p = (cv2.transform(np.array([p.T], copy=True).astype(np.float32), T)[0]).T
		Tr = T @ Tr
	return Tr
