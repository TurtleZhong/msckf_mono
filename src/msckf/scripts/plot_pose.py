#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class RosPlot(object):
    def __init__(self, ax, name):
        self.ax = ax
        self.hl, = ax.plot([], [], label=name)
        self.xdata = []
        self.ydata = []

    def update(self, x, y):
        self.xdata.append(x)
        self.ydata.append(y)
        self.hl.set_xdata(self.xdata)
        self.hl.set_ydata(self.ydata)
        plt.show()


def update_line(hl, x, y):
    hl.set_xdata(np.append(hl.get_xdata(), x))
    hl.set_ydata(np.append(hl.get_ydata(), y))
    plt.pause(0.1)

# while True:sg
#     y = np.random.random()
    # update_line(hl, time.time() - init, y)

def pose_callback(msg, hl, datum):
    # msg = PoseWithCovarianceStamped()
    print("got pose {}, {}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
    hl.update(msg.pose.pose.position.x, msg.pose.pose.position.y)

def odom_callback(msg, callback_args):
    hl = callback_args[0]
    datum = callback_args[1]
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if not datum:
        datum.extend([x,y])

    x = x - datum[0]
    y = y - datum[1]
    #print("got msg {}, {}".format(x, y))
    hl.update(x, y)


def plot_error(gps_h, msckf_h, imu_pos_h):
    fig = plt.figure(2)
    gps_pos = np.column_stack((gps_h.xdata, gps_h.ydata))
    msckf_pos = np.column_stack((msckf_h.xdata, msckf_h.ydata))
    imu_only_pos = np.column_stack((imu_pos_h.xdata, imu_pos_h.ydata))

    L = np.min([len(gps_pos), len(msckf_pos), len(imu_only_pos)])
    gps_pos = gps_pos[1:L]
    msckf_pos = msckf_pos[1:L]
    imu_only_pos = imu_only_pos[1:L]

    mskcf_error = np.linalg.norm(gps_pos - msckf_pos, axis=1)
    imu_error = np.linalg.norm(gps_pos - imu_only_pos, axis=1)



if __name__ == "__main__":
    plt.ion()
    fig = plt.figure(facecolor="white")
    ax = fig.add_subplot(111)
    #hl = RosPlot(ax)
    hpose_gps = RosPlot(ax, 'GPS')
    hpose_ekf = RosPlot(ax, 'MSKCF')
    hpose_imu= RosPlot(ax, 'IMU only')


    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.axis([-100, 100, -100, 100])

    plt.legend([hpose_gps.hl, hpose_imu.hl, hpose_ekf.hl], ['GPS', 'IMU only', 'MSKCF'])

    rospy.init_node('plot_pose')
    datum = []

    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback, (hpose_gps, []))
    #pose_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, odom_callback, (hpose_gps, []))
    imu_pose_sub = rospy.Subscriber("/imu_only_odom", PoseWithCovarianceStamped, odom_callback, (hpose_imu, []))
    leo_pose_sub = rospy.Subscriber("/odom_combined", PoseWithCovarianceStamped, odom_callback, (hpose_ekf, []))

    def timer_callback(evt):
        plot_error(hpose_gps, hpose_ekf, hpose_imu)


    rospy.Timer(rospy.Duration(1.0), timer_callback, oneshot=False)




#rospy.spin()
    plt.show(block=True)