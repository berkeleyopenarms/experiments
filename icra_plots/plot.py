import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import helpers
import csv
from ssp import make_circle
from tf.transformations import euler_from_quaternion
from operator import itemgetter

# DATA_DIRECTORY = "pp_data"
DATA_DIRECTORY = "optitrack/full_data"
# DATA_DIRECTORY = "ah"
# DATA_DIRECTORY = "optitrack/not_full"
# DATA_DIRECTORY = "optitrack"
# DATA_DIRECTORY = "data_icra/pp/8-29-noi"
# DATA_DIRECTORY = "quigley_data/9-4-duty_cycle-post-david-tuning"

def euler_from_quat(quaternion):
    return euler_from_quaternion(quaternion) # Returns in radians

def calc_statistics(poses, name):
    x = [] #initialize list of x vive positions
    y = [] # '' y ''
    z = [] # '' z ''
    roll = [] # '' roll ''
    pitch = [] # '' pitch ''
    yaw = [] # '' yaw ''
    for p in poses:
        x.append(p[0])
        y.append(p[1])
        z.append(p[2])
        # print(p)
        euler = euler_from_quat(p[3:])
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])

    stats = {
        'x_std':np.std(np.array(x)),
        'y_std':np.std(np.array(y)),
        'z_std':np.std(np.array(z)),
        'roll_std':np.std(np.array(roll)),
        'pitch_std':np.std(np.array(pitch)),
        'yaw_std':np.std(np.array(yaw))}

    with open(name + '.csv', 'wb') as csv_file:
        writer = csv.writer(csv_file)
        for key, value in stats.items():
            print("{}: {}".format(key, value))
            writer.writerow([key, value])

def load_from_csv(path):
    raw = np.genfromtxt(str(path), delimiter=',',dtype=str)
    labels = raw[:1]
    data = raw[1:].astype(np.float)
    return labels, data

def get_intervals(cmd, cmd_data, rel_data):
    intervals = []
    end_pose = 0
    joints = cmd_data[:,-7:]
    for i, j in enumerate(joints[:-1]):
        if np.sum((cmd - j)**2) < 1.0e-4 and 1.0 < cmd_data[i+1,0] - cmd_data[i,0]:
            intervals.append([cmd_data[i,0], cmd_data[i+1,0]])

            end_pose = cmd_data[i,1:]

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
    return data, intervals, end_pose[0:7]

def find_closest_time(time, data):
    data_time = np.array(data)[:,0]
    diff = data_time - time
    diff = diff * diff;
    diff = diff.tolist()

    ind = diff.index(min(diff))

    return data[ind]

def match_points(to_be_matched, to_be_matched_from):

    matches = []
    index = 0
    for point in to_be_matched:
        time = point[0]
        while True:
            time_current = to_be_matched_from[index][0]
            if index == len(to_be_matched_from) - 1:
                matches.append(to_be_matched_from[index])
                break
            time_next = to_be_matched_from[index + 1][0]

            if abs(time - time_current) < abs(time - time_next):
                matches.append(to_be_matched_from[index])
                break

            index += 1

    return np.asarray(matches)

def match_points_time(to_be_matched, to_be_matched_from):

    matches = []
    index = 0
    for point in to_be_matched:
        time = point[0]
        while True:
            time_current = to_be_matched_from[index][0]
            if index == len(to_be_matched_from) - 1:
                matches.append(to_be_matched_from[index])
                break
            time_next = to_be_matched_from[index + 1][0]

            if time - time_current < time - time_next:
                matches.append(to_be_matched_from[index])
                break

            index += 1

    return np.asarray(matches)

def plot_dist_vs_err():
    _, d_p = load_from_csv("dist_prec.csv")
    # print(loc.shape)
    # print(data.shape)
    # print(vive.shape)
    data = []
    for i in range(8):
        data.append([d_p[0,i], d_p[1,i], d_p[2,i]])
    def func_d(i):
        return i[0]
    data.sort(key=func_d)
    data = np.array(data)
    print(data)
    plt.plot(data[:,0], data[:,1], label="circle_dist vs distance from home_pos")
    plt.plot(data[:,0], data[:,2], label="circle_avg vs distance from home_pos")
    plt.title("home pos")
    plt.legend()
    plt.savefig("err_vs_dist_from_home_pos")
    plt.show()

    data = []
    for i in range(8):
        data.append([d_p[3,i], d_p[1,i], d_p[2,i]])
    def func_d(i):
        return i[0]
    data.sort(key=func_d)
    data = np.array(data)
    print(data)
    plt.plot(data[:,0], data[:,1], label="circle_dist vs distance from base frame")
    plt.plot(data[:,0], data[:,2], label="circle_avg vs distance from base frame")
    plt.title("base_frame")
    plt.legend()
    plt.savefig("err_vs_dist_from_base_frame")
    plt.show()

def plot_points(loc, data, vive, name="plot", home=0):
    # print(loc.shape)
    # print(data.shape)
    # print(vive.shape)
    start_vive, _, end_pose = get_intervals(loc, data, vive)
    print(end_pose)
    print("dist_from_home_pos: {}".format(np.linalg.norm(home[0:3] - end_pose[0:3])))
    print("dist_from_home_frame: {}".format(np.linalg.norm(end_pose[0:3])))
    # print(start_vive[0].shape)
    poses = [s[-1][1:] for s in start_vive]
    calc_statistics(poses, name)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for x in start_vive:
        data_xyz = x[::40,1:4].T
        ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)
    # plt.show()
    return np.array(poses)

############## plotting code ############################################
def pc_to_brent():
    _, cmd = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    _, vive = load_from_csv(DATA_DIRECTORY + "/vive.csv")
    _, fk = load_from_csv(DATA_DIRECTORY + "/ee.csv")
    # _, start_end = load_from_csv(DATA_DIRECTORY + "/start_end_joints.csv")
    print("Data loaded")

    # Brent's transform code
    p_all = vive[100:-100]
    q_all = cmd[100:-100]

    p = p_all[::1]
    # q = np.asarray([find_closest_time(x[0], q_all) for x in p])
    q = match_points(p, q_all)
    print("Points matched")
    # return
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
    start_vive, intervals, _ = get_intervals(start_end[0], data, vive)

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

# def plot_quigley():
    # _, data = load_from_csv("q_data/cmd.csv")
    # _, vive = load_from_csv("q_data/vive.csv")
    # _, start_end = load_from_csv("q_data/home.csv")
    # plot_points(start_end[0], data, vive, name="quigley")

def plot_quigley():
    _, data = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    _, vive = load_from_csv(DATA_DIRECTORY + "/vive.csv")
    _, start_end = load_from_csv(DATA_DIRECTORY + "/home.csv")
    _, other_8 = load_from_csv("other_8.csv")
    count = 0

    start_vive, _, home_pose = get_intervals(start_end[0], data, vive)

    for d in other_8:
        print("point number {}".format(count))
        p_t = "other_quig" + str(count)
        poses = plot_points(other_8[count], data, vive, name=p_t, home=home_pose)
        xyz_mat = poses[:,0:3]
        u, s, vh = np.linalg.svd(xyz_mat, full_matrices=True)
        points_svd = (u[:,:3]*s).dot(vh[:,0:3])
        save_plot(points_svd[:,0], points_svd[:,1], p_t + "_svd_pc1_pc2.eps")
        # save_plot(poses[:,0],poses[:,1], p_t + "_xy")
        # save_plot(poses[:,0],poses[:,2], p_t + "_xz")
        # save_plot(poses[:,1],poses[:,2], p_t + "_yz")
        count+=1
        print(" ")
    return


    p_t = "quigley"

    poses = plot_points(start_end[0], data, vive, name=p_t)
    # print(poses)
    save_plot(poses[:,0],poses[:,1], p_t + "_xy")
    save_plot(poses[:,0],poses[:,2], p_t + "_xz")
    save_plot(poses[:,1],poses[:,2], p_t + "_yz")

    xyz_mat = poses[:,0:3]
    xyz_mat = xyz_mat - np.mean(xyz_mat, axis=0)
    # print("mean", np.mean(xyz_mat, axis=0))
    # print("sum", np.sum(xyz_mat, axis=0))

    u, s, vh = np.linalg.svd(xyz_mat, full_matrices=True)
    # print(u.shape)
    # print(s.shape)
    # print(vh.shape)
    print(s)
    s[2] = 0.0
    print(vh)
    print(np.cross(vh[0,:], vh[1,:]))
    # s[1] = 0.0
    points_svd = (u[:,:3]*s).dot(vh[:,0:3])
    save_plot(points_svd[:,0], points_svd[:,1], p_t + "_svd_pc1_pc2")

def save_plot(x, y, title):
    fig = plt.figure()
    x_u = np.mean(x)
    y_u = np.mean(y)
    dist_arry = []

    p_list = [(x_i, y_i) for x_i, y_i in zip(x,y)]



    c = make_circle(p_list)
    # print(c)
    x -= c[0]
    y -= c[1]
    # x -= x_u
    # y -= y_u

    for x_i, y_i in zip(x, y):
        dist = np.sqrt((x_i)**2 + (y_i)**2)
        dist_arry.append(dist)

    tuple_points = [(d, x_i, y_i) for d, x_i, y_i in zip(dist_arry, x, y)]
    def func_d(i):
        return i[0]
    tuple_points.sort(key=func_d)
    numpoints = len(tuple_points)
    percent = 0.9
    dist_p = []
    print("num_points: " + str(numpoints))
    for i in range(int(numpoints*percent)):
        x_i = tuple_points[i][1]
        y_i = tuple_points[i][2]
        dist = np.sqrt((x_i)**2 + (y_i)**2)
        dist = tuple_points[i][0]
        dist_p.append(dist)
    # plt.plot(dist_p)
    # plt.show()

    # print(tuple_points)

    circle= plt.Circle((0,0), radius=1000*max(dist_arry), color='r', fill=False)
    circle2= plt.Circle((0,0), radius=1000*(np.average(dist_arry)), color='b', fill=False)
    ax=fig.gca()
    ax.add_patch(circle)
    ax.add_patch(circle2)
    print("max_all: {}".format(max(dist_arry)))
    print("average_dis: {}".format(np.average(dist_arry)))
    # print("max_pec: {}".format(max(dist_p)))
    # print("std x")
    # print(np.std(x))
    # print("std y")
    # print(np.std(y))
    # print("MSE")
    # sq = np.square(dist_arry)
    # print(np.average(sq))
    # print(np.sqrt(np.average(sq)))

    # circle= plt.Circle((0,0), radius=0.1, color='r', fill=False)
    # ax=fig.gca()
    # ax.add_artist(circle)

    # print("blah blah blah", x.shape)
    plt.plot(x * 1000.0, y * 1000.0, "x")
    plt.title(title)
    plt.xlabel('mm')
    plt.ylabel('mm')
    axis=plt.gca()
    axis.set_xlim([-2.5, 2.5])
    axis.set_ylim([-2.5, 2.5])
    axis.set_xticks([x / 10.0 for x in range(-25, 30, 5)])
    axis.set_yticks([x / 10.0 for x in range(-25, 30, 5)])
    axis.set_aspect('equal')
    fig.savefig("lim_"+ title, format='eps', dpi=5000)

def plot_vive():
    _, data = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    _, vive = load_from_csv(DATA_DIRECTORY + "/vive.csv")
    _, start_end = load_from_csv(DATA_DIRECTORY + "/start_end_joints.csv")

    p_t = "start"
    poses = plot_points(start_end[0], data, vive, name=p_t)
    print(poses)
    save_plot(poses[:,0],poses[:,1], p_t + "_xy")
    save_plot(poses[:,0],poses[:,2], p_t + "_xz")
    save_plot(poses[:,1],poses[:,2], p_t + "_yz")

    p_t = "end"
    poses = plot_points(start_end[1], data, vive, name="end")
    save_plot(poses[:,0],poses[:,1], p_t + "_xy")
    save_plot(poses[:,0],poses[:,2], p_t + "_xz")
    save_plot(poses[:,1],poses[:,2], p_t + "_yz")

def plot_err():
    _, ee = load_from_csv(DATA_DIRECTORY + "/ee.csv")
    _, cmd = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    q = match_points(ee, cmd)
    err = np.abs(q - ee)

    plt.plot(q[:,0], err[:,8], label="j0")
    plt.plot(q[:,0], err[:,9], label="j1")
    plt.plot(q[:,0], err[:,10], label="j2")
    plt.plot(q[:,0], err[:,11], label="j3")
    plt.plot(q[:,0], err[:,12], label="j4")
    plt.plot(q[:,0], err[:,13], label="j5")
    plt.plot(q[:,0], err[:,14], label="j6")
    plt.title("error")
    plt.legend()
    plt.show()

def integrate(t, y):
    num = len(t)
    array = [0]
    for i in range(num-1):
        array.append(array[i] + (t[i+1]-t[i]) * (y[i]) )
    return t, y, np.array(array)

def plot_integrate(t, y):
    num = len(t)
    array = [0]
    for i in range(num-1):
        array.append(array[i] + (t[i+1]-t[i]) * (y[i]) )

    fig, ax1 = plt.subplots()

    ax1.plot(t, array, label="integrated")

    ax2 = ax1.twinx()
    # ax2.plot(t, s2, 'r.')
    # ax2.set_ylabel('sin', color='r')

    plt.plot(t, y, color='r', label="orig")

    plt.title("power_integrated")
    # plt.legend()
    # plt.show()
    fig.savefig("heat_power.eps", format='eps', dpi=5000)

def plot_curr():
    # plot_err()
    _, curr = load_from_csv(DATA_DIRECTORY + "/motor.csv")
    st = curr[0,0]
    curr[:,0] = curr[:,0] - st
    curr = curr[0:8000,:]
    plt.plot(curr[:,0], curr[:,1], label="m0")
    plt.plot(curr[:,0], curr[:,2], label="m1")
    plt.plot(curr[:,0], curr[:,3], label="m2")
    plt.plot(curr[:,0], curr[:,4], label="m3")
    plt.plot(curr[:,0], curr[:,5], label="m4")
    # plt.plot(curr[:,0], curr[:,6], label="m5")
    # plt.plot(curr[:,0], curr[:,7], label="m6")
    plt.title("current")
    plt.legend()
    # plt.show()

    csq = 18 * np.multiply(curr, curr)
    fig = plt.figure()

    plt.plot(curr[:,0], csq[:,1], label="m0")
    plt.plot(curr[:,0], csq[:,2], label="m1")
    plt.plot(curr[:,0], csq[:,3], label="m2")
    plt.plot(curr[:,0], csq[:,4], label="m3")
    plt.plot(curr[:,0], csq[:,5], label="m4")

    power = csq[:,1] + csq[:,2] + csq[:,3] + csq[:,4] + csq[:,5]
    plt.plot(curr[:,0], power, label="all")
    # plt.plot(curr[:,0], curr[:,6], label="m5")
    # plt.plot(curr[:,0], curr[:,7], label="m6")
    plt.title("power")
    plt.legend()
    fig.savefig("power.eps", format='eps', dpi=5000)
    # plt.show()

    t, m0, im0 = integrate(curr[:,0], csq[:,1])
    t, m1, im1 = integrate(curr[:,0], csq[:,2])
    t, m2, im2 = integrate(curr[:,0], csq[:,3])
    t, m3, im3 = integrate(curr[:,0], csq[:,4])
    t, m4, im4 = integrate(curr[:,0], csq[:,5])
    t, m5, im5 = integrate(curr[:,0], csq[:,6])
    t, m6, im6 = integrate(curr[:,0], csq[:,7])

    fig, ax1 = plt.subplots()

    ax1.plot(t, im0, label="integrated m0")
    ax1.plot(t, im1, label="integrated m1")
    ax1.plot(t, im2, label="integrated m2")
    ax1.plot(t, im3, label="integrated m3")
    ax1.plot(t, im4, label="integrated m4")
    ax1.plot(t, im5, label="integrated m5")
    ax1.plot(t, im6, label="integrated m6")
    ax2 = ax1.twinx()
    # ax2.plot(t, s2, 'r.')
    # ax2.set_ylabel('sin', color='r')

    ax2.plot(t, m0, label="power m0")
    ax2.plot(t, m1, label="power m1")
    ax2.plot(t, m2, label="power m2")
    ax2.plot(t, m3, label="power m3")
    ax2.plot(t, m4, label="power m4")
    ax2.plot(t, m5, label="power m5")
    ax2.plot(t, m6, label="power m6")
    fig.savefig("all_heat_power.eps", format='eps', dpi=5000)
    # plt.plot(t, y, color='r', label="orig")

    plt.title("power_integrated")
    plt.legend()
    # plt.show()
##########################################################
    fig, ax1 = plt.subplots()

    ax1.plot(t, m0, label="power m0")
    ax1.plot(t, m1, label="power m1")
    ax1.plot(t, m2, label="power m2")
    ax1.plot(t, m3 + m4 + m5 + m6, label="power m3, m4, m5, m6")
    ax1.set_ylabel('Power (Watt)', color='r')

    ax2 = ax1.twinx()
    # ax2.plot(t, s2, 'r.')
    # ax2.set_ylabel('sin', color='r')

    ax2.plot(t, im0, label="integrated m0")
    ax2.plot(t, im1, label="integrated m1")
    ax2.plot(t, im2, label="integrated m2")
    ax2.plot(t, im3 + im4 + im5 + im6, label="power m3, m4, m5, m6")
    ax2.set_ylabel('Joule (Watt * Seconds)', color='r')
    # plt.plot(t, y, color='r', label="orig")

    plt.title("just 3 power_integrated")
    plt.legend()
    # plt.show()
    fig.savefig("3_heat_power.eps", format='eps', dpi=5000)

    # plot_integrate(curr[:,0], power)
    fig, ax1 = plt.subplots()

    ax1.plot(t, m0, label="power m0")
    ax1.plot(t, m1, label="power m1")
    ax1.plot(t, m2, label="power m2")
    ax1.plot(t, m3 + m4 + m5 + m6, label="power m3, m4, m5, m6")
    ax1.set_ylabel('Power (Watt)', color='r')

    ax2 = ax1.twinx()
    # ax2.plot(t, s2, 'r.')
    # ax2.set_ylabel('sin', color='r')

    ax2.plot(t, im0, label="integrated m0")
    ax2.plot(t, im1, label="integrated m1")
    ax2.plot(t, im2, label="integrated m2")
    ax2.plot(t, im3 + im4 + im5 + im6, label="power m3, m4, m5, m6")
    ax2.set_ylabel('Joule (Watt * Seconds)', color='r')
    # plt.plot(t, y, color='r', label="orig")

    plt.title("no legend just 3 power_integrated")
    # plt.legend()
    # plt.show()
    fig.savefig("no_legend_3_heat_power.eps", format='eps', dpi=5000)


def plot_traj():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    _, data = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    data_xyz = data[::40,1:4].T
    ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)

    _, data = load_from_csv(DATA_DIRECTORY + "/ee.csv")
    data_xyz = data[::40,1:4].T
    ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)
    plt.show()

# fig = plt.figure()
# circle= plt.Circle((0,0), radius=0.1, color='r', fill=False)
# ax=fig.gca()
# ax.add_artist(circle)
# plt.title("hi")
# plt.show()

# plot_vive()
# pc_to_brent()
# plot_dist_vs_err()
plot_quigley()
# plot_traj()
# plot_curr()
