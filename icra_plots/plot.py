import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_from_csv(path):
    raw = np.genfromtxt(str(path), delimiter=',',dtype=str)
    labels = raw[:1]
    data = raw[1:].astype(np.float)
    return labels, data


# time,x,y,z,qx,qy,qz,qw,j0,j1,j2,j3,j4,j5,j6
_, ee_data = load_from_csv("ee.csv")

# time,x,y,z,qx,qy,qz,qw
# _, ee_data = load_from_csv("vive.csv")

ee_data_xyz = ee_data[::20,1:4].T
print(ee_data_xyz.shape)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(ee_data_xyz[0], ee_data_xyz[1], ee_data_xyz[2], linewidth=1)
plt.show()
