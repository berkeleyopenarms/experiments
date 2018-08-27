import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_from_csv(path):
    raw = np.genfromtxt(str(path), delimiter=',',dtype=str)
    labels = raw[:1]
    data = raw[1:].astype(np.float)
    return labels, data


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

_, data = load_from_csv("data/cmd.csv")
data_xyz = data[::40,1:4].T
ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)

_, data = load_from_csv("data/ee.csv")
data_xyz = data[::40,1:4].T
ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)

plt.show()
