#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt 

x = []
y = []
def disply(data:PoseStamped):
    plt.clf()
    x1 = data.pose.position.x
    y1 = data.pose.position.y
    x.append(x1)
    y.append(y1)

    plt.scatter(x,y)

    plt.pause(0.00001)
    

if __name__=='__main__':
    try:
        rospy.init_node("display_position")
        plt.ion()
        rospy.Subscriber("/pose", PoseStamped, disply)
        plt.ioff()
        plt.show()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass