#! /usr/bin/env python3

import os 
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped
def make_cmd_vel(x=0,y=0,z=0,r=0,p=0,yaw=0):
    global vel
    msg = Twist()
    
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z

    msg.angular.x = r
    msg.angular.y = p
    msg.angular.z = yaw

    return msg


if __name__=='__main__':
    try:

        rospy.init_node('starting')
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        msg = make_cmd_vel()
        cmd_pub.publish(msg)

        for i in range(5):
            time.sleep(1)
            print(i)
            cmd_pub.publish(msg)
        os.system('rostopic pub /start_go std_msgs/Bool "data: true"')
        print('start go ')

    except rospy.ROSInterruptException:
        pass