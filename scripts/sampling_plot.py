import rospy
from geometry_msgs.msg import Pose2D

from matplotlib import pyplot as plt
import matplotlib.animation as animation
from scipy import ndimage
import math


pose = Pose2D()


def callback(data):
    global pose
    pose.x = data.x
    pose.y = data.y
    pose.theta = data.theta
    # update_plot(0)

def odom_ploter():
    rospy.init_node('sampling_plot', anonymous=True)
    rospy.Subscriber("odom_data", Pose2D, callback)

    fig, ax = plt.subplots()
    ax.set_xlim(-1000, 1000)
    ax.set_ylim(-1000, 1000)
    ax.grid(True)

    # scat = plt.scatter(0, 0)
    ani = animation.FuncAnimation(fig, update_plot)
    plt.show()

    rospy.spin()

def update_plot(i):
    colors = (1,0,0)
    scat = plt.scatter(pose.x, pose.y, s=1, c=colors, alpha=1)
    rospy.loginfo("x: {}, y: {}".format(pose.x, pose.y))
    return scat, 


if __name__ == '__main__':
    try:
        odom_ploter()
    except rospy.ROSInterruptException:
        pass