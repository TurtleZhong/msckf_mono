#! /usr/bin/env python
# Load the CSV files saved by test_sim, and plot the data
# Give the name of the save directory as a command line argument

import sys
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def process_files(dir_name):
    sim_data = np.loadtxt(
        dir_name + "/parameters.csv",
        dtype={'names': ('duration', 'dt', 'n_features', 'vel_stddev', 'rot_vel_stddev', 'camera_stddev'),
               'formats': ('f', 'f', 'i', 'f', 'f', 'f')},
        skiprows=1,
        delimiter=",")

    motion_data = np.loadtxt(
        dir_name + "/motion.csv",
        dtype={'names': ('time', 'pos', 'rot', 'vel', 'rot_vel', 'vel_input', 'rot_vel_input'),
               'formats': ('f', '3f', '3f', '3f', '3f', '3f', '3f')},
        skiprows=1,
        delimiter=",")

    msckf_data = np.loadtxt(
        dir_name + "/msckf.csv",
        dtype={'names': ('time', 'pos', 'rot', 'cov_pos', 'cov_rot'),
               'formats': ('f', '3f', '3f', '3f', '3f')},
        skiprows=1,
        delimiter=",")

    imu_only_data = np.loadtxt(
        dir_name + "/imu_only.csv",
        dtype={'names': ('time', 'pos', 'rot', 'cov_pos', 'cov_rot'),
               'formats': ('f', '3f', '3f', '3f', '3f')},
        skiprows=1,
        delimiter=",")

    feature_data = np.loadtxt(
        dir_name + "/features.csv",
        dtype={'names': ('id', 'pos'),
               'formats': ('i', '3f')},
        skiprows=1,
        delimiter=",")

    plot_motion(motion_data, msckf_data, imu_only_data, feature_data)
    plot_error(motion_data, msckf_data, imu_only_data, feature_data)
    plot_error_in_z(motion_data, msckf_data, imu_only_data, feature_data)

    font = {'family' : 'normal',
            'weight' : 'bold',
            'size'   : 14}

    matplotlib.rc('font', **font)


def plot_motion(motion_data, msckf_data, imu_only_data, feature_data):
    fig = plt.figure(facecolor="white")

    pos = motion_data['pos'].T
    msckf_pos = msckf_data['pos'].T
    imu_pos = imu_only_data['pos'].T

    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*pos, linewidth=2.0, label='Simulation')
    ax.plot(*msckf_pos, linewidth=2.0, label='MSCKF')
    ax.plot(*imu_pos, linewidth=2.0, label='IMU only')

    # plot only near features
    feature_pos = np.array([f for f in feature_data['pos']
                            if f[0] < 200 and np.abs(f[1]) < 100 and np.abs(f[2]) < 100])

    ax.plot(*feature_pos.T, marker='.', linestyle='none', markersize=2, color='c')
    plt.legend()

    set_axes_equal(ax)
    fig.tight_layout()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')


def plot_error_in_z(motion_data, msckf_data, imu_only_data, feature_data):
    fig = plt.figure(facecolor="white")

    t = motion_data['time']
    pos = motion_data['pos'].T
    msckf_pos = msckf_data['pos'].T
    msckf_pos_d = np.sqrt(msckf_data['cov_pos'].T)

    imu_pos = imu_only_data['pos'].T
    imu_pos_d = np.sqrt(imu_only_data['cov_pos'].T)

    ax = fig.add_subplot(111)
    ax.plot(t, pos[2], label='Simulation',
            linewidth=2.0, color='b')


    ax.fill_between(t, imu_pos[2] - imu_pos_d[2], imu_pos[2] + imu_pos_d[2],
                    linewidth=0, facecolor='#eacdc9', alpha=0.5)
    ax.plot(t, imu_pos[2], label='IMU only',
            linewidth=2.0, color='r')

    ax.fill_between(t, msckf_pos[2] - msckf_pos_d[2], msckf_pos[2] + msckf_pos_d[2],
                    linewidth=0, facecolor='#c7e5bc', alpha=0.5)
    ax.plot(t, msckf_pos[2], label='MSCKF',
            linewidth=2.0, color='g')

    plt.legend()

    ax.set_xlabel('Time (s)', fontsize=20)
    ax.set_ylabel('Z Position (m)', fontsize=20)


def plot_error(motion_data, msckf_data, imu_only_data, feature_data):
    fig = plt.figure(facecolor="white")

    t = motion_data['time']
    pos = motion_data['pos']
    msckf_pos = msckf_data['pos']
    msckf_pos_error = np.linalg.norm(pos - msckf_pos, axis=1)


    imu_pos = imu_only_data['pos']
    imu_pos_error = np.linalg.norm(pos - imu_pos, axis=1)

    ax = fig.add_subplot(111)

    ax.plot(t, imu_pos_error, label='IMU only pos error',
            linewidth=2.0, color='r')

    ax.plot(t, msckf_pos_error, label='MSCKF pos error',
            linewidth=2.0, color='g')

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 40])
    plt.legend()

    ax.set_xlabel('Time (s)', fontsize=20)
    ax.set_ylabel('Error (m)', fontsize=20)


def set_axes_equal(ax, scale=1):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
      
    From: http://stackoverflow.com/a/31364297/431033
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = scale * 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    # Fewer ticks
    ax.set_xticks(np.linspace(x_middle - plot_radius, x_middle + plot_radius, 5))
    ax.set_yticks(np.linspace(y_middle - plot_radius, y_middle + plot_radius, 5))
    ax.set_zticks(np.linspace(z_middle - plot_radius, z_middle + plot_radius, 5))



if __name__ == "__main__":
    # Expect directory name as first argument
    dir_name = sys.argv[1]
    process_files(dir_name)
    plt.show()
