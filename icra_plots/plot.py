import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import helpers
import csv
from tf.transformations import euler_from_quaternion

# DATA_DIRECTORY = "pp_data"
DATA_DIRECTORY = "optitrack/full_data"
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
        print(p)
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
    joints = cmd_data[:,-7:]
    for i, j in enumerate(joints[:-1]):
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


def plot_points(loc, data, vive, name="plot"):
    print(loc.shape)
    print(data.shape)
    print(vive.shape)
    start_vive, _ = get_intervals(loc, data, vive)
    print(start_vive[0].shape)
    poses = [s[-1][1:] for s in start_vive]
    calc_statistics(poses, name)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for x in start_vive:
        data_xyz = x[::40,1:4].T
        ax.plot(data_xyz[0], data_xyz[1], data_xyz[2], linewidth=1)
    plt.show()
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
    plot_points(start_end[0], data, vive, name="quigley")

def plot_quigley():
    _, data = load_from_csv(DATA_DIRECTORY + "/cmd.csv")
    _, vive = load_from_csv(DATA_DIRECTORY + "/vive.csv")
    _, start_end = load_from_csv(DATA_DIRECTORY + "/home.csv")

    p_t = "quigley"
    poses = plot_points(start_end[0], data, vive, name=p_t)
    print(poses)
    save_plot(poses[:,0],poses[:,1], p_t + "_xy")
    save_plot(poses[:,0],poses[:,2], p_t + "_xz")
    save_plot(poses[:,1],poses[:,2], p_t + "_yz")

def save_plot(x, y, title):
    fig = plt.figure()
    x -= np.min(x)
    y -= np.min(y)
    print("blah blah blah", x.shape)
    plt.plot(x * 1000.0, y * 1000.0, "x")
    plt.title(title)
    fig.savefig(title)

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

def plot_curr():
    plot_err()
    _, curr = load_from_csv(DATA_DIRECTORY + "/motor.csv")
    plt.plot(curr[:,0], curr[:,1], label="m0")
    plt.plot(curr[:,0], curr[:,2], label="m1")
    plt.plot(curr[:,0], curr[:,3], label="m2")
    plt.plot(curr[:,0], curr[:,4], label="m3")
    plt.plot(curr[:,0], curr[:,5], label="m4")
    plt.plot(curr[:,0], curr[:,6], label="m5")
    plt.plot(curr[:,0], curr[:,7], label="m6")
    plt.title("current")
    plt.legend()
    plt.show()


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

# plot_vive()
pc_to_brent()
plot_quigley()
# plot_traj()
plot_curr()
