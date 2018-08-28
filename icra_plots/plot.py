import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import helpers

def load_from_csv(path):
    raw = np.genfromtxt(str(path), delimiter=',',dtype=str)
    labels = raw[:1]
    data = raw[1:].astype(np.float)
    return labels, data

def get_intervals(cmd, cmd_data, rel_data):
    intervals = []
    joints = cmd_data[:,-7:]
    for i, j in enumerate(joints):
        if np.sum((cmd - j)**2) < 1.0e-4 and 1.0 < cmd_data[i+1,0] - cmd_data[i,0]:
            intervals.append([cmd_data[i,0], cmd_data[i+1,0]])
            # print(cmd_data[i,0]-cmd_data[i+1,0])
            # print(cmd_data[i,0], cmd_data[i+1,0])
    intervals = np.array(intervals)

    data = []
    for i in intervals:
        vive_int = []
        for d in rel_data:
            if d[0] >= i[0] and d[0] <= i[1]:
                vive_int.append(d)
        data.append(np.array(vive_int))
    return data, intervals;

def find_closest_time(time, data):
    data_time = np.array(data)[:,0]
    diff = data_time - time
    diff = diff * diff;
    diff = diff.tolist()

    ind = diff.index(min(diff))

    return data[ind]


def plot_points(loc, data, vive):

    start_vive = get_intervals(loc, data, vive)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for x in start_vive:
        data_xyz = x[::40,1:4].T
        ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)
    plt.show()

############## plotting code ############################################

def pc_to_brent():
    _, data = load_from_csv("pp_data/cmd.csv")
    _, vive = load_from_csv("pp_data/vive.csv")
    _, fk = load_from_csv("pp_data/ee.csv")
    _, start_end = load_from_csv("pp_data/start_and_end_joints.csv")

    # Brent's transform code
    p_all = vive[100:-100]
    q_all = fk[100:-100]

    p = p_all[::1]
    q = np.asarray([find_closest_time(x[0], q_all) for x in p])
    p_xyz = p[:,1:4]
    q_xyz = q[:,1:4]

    R, t, error = helpers.find_optimal_transform(p_xyz, q_xyz)
    print(R.dot(np.asarray([1,2,3])))

    print(q_xyz.shape)
    p_xyz = R.dot(p_xyz.T).T + t

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(p_xyz.T[0], p_xyz.T[1], p_xyz.T[2])
    ax.plot(q_xyz.T[0], q_xyz.T[1], q_xyz.T[2])
    plt.show()

    return


    # Philipp's interval code
    start_vive, intervals = get_intervals(start_end[0], data, vive)

    data_qwe = []
    for i in intervals:
        vive_int = []
        for d in fk:
            if d[0] >= i[0] and d[0] <= i[1]:
                vive_int.append(d)
        data_qwe.append(np.array(vive_int))
    points = [ x[-1] for x in data_qwe]

    vive_points = []
    ee_fk_points = []
    for p in points:
    # for p in vive[::1000]:
        vive_points.append(p[1:4])
        ee_fk_points.append(find_closest_time(p[0], data)[1:4])

    vive_points = np.asarray(vive_points).T
    ee_fk_points = np.asarray(ee_fk_points).T
    print(ee_fk_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.plot(vive_points[0], vive_points[1], vive_points[2])
    ax.scatter(ee_fk_points[0], ee_fk_points[1], ee_fk_points[2])
    plt.show()

    # return
    # print(ee_fk_points)
    return vive_points, ee_fk_points

def plot_quigley():
    _, data = load_from_csv("q_data/cmd.csv")
    _, vive = load_from_csv("q_data/vive.csv")
    _, start_end = load_from_csv("q_data/home.csv")
    plot_points(start_end[0], data, vive)


def plot_vive():
    _, data = load_from_csv("pp_data/cmd.csv")
    _, vive = load_from_csv("pp_data/vive.csv")
    _, start_end = load_from_csv("pp_data/start_and_end_joints.csv")
    plot_points(start_end[0], data, vive)
    plot_points(start_end[1], data, vive)

def plot_traj():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    _, data = load_from_csv("pp_data/cmd.csv")
    data_xyz = data[::40,1:4].T
    ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)

    _, data = load_from_csv("pp_data/ee.csv")
    data_xyz = data[::40,1:4].T
    ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)
    plt.show()

# plot_vive()
pc_to_brent()
# plot_quigley()
# plot_traj()
