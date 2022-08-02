import numpy as np
import matplotlib.pyplot as plt

pi = np.pi
cos = np.cos
sin = np.sin

N = 1000

t = np.arange(N)/N

target = np.vstack( (cos(2*pi*t), 2*sin(2*pi*t)) ) + np.random.normal(0, 1e-2, size = (2, N))

theta = pi/3

x_s = 1.25
y_s = -0.50

T = np.array(
    [
        [cos(theta), -sin(theta), x_s],
        [sin(theta), cos(theta), y_s],
        [0, 0, 1]
    ]
)

source = (T @ np.vstack((target, np.ones(N))))[:2, :] + np.random.normal(0, 1e-2, size = (2, N))

np.savetxt('/home/schmidd/Documents/Github/anytime-stack/algorithms/anytime_cpp/src/icp/target.csv', target, delimiter=',')

np.savetxt('/home/schmidd/Documents/Github/anytime-stack/algorithms/anytime_cpp/src/icp/source.csv', source, delimiter=',')

plt.figure()

plt.plot(target[0], target[1], lw = 2.5, linestyle = None, marker = 'o')

plt.plot(source[0], source[1], lw = 2.5, linestyle = None, marker = 'o')

plt.grid()

plt.axis('equal')

plt.show()