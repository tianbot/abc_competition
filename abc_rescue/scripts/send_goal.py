#!/usr/bin/env python
# date: 2023年11月23日15:18:19
# author: Lr_2002(Xianhao Wang)

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, Quaternion

def send_goal(x=1.67, y=0.67):
    """send goal point to the /tianbot_mini/move_base_simple/goal ""

    Args:
        x (float, optional): the x of the goal position. Defaults to 1.67.
        y (float, optional): the y of the goal position. Defaults to 0.67.
    """
    # Initialize the ROS node
    rospy.init_node('send_move_base_simple_goal_node', anonymous=True)

    # Create a PoseStamped message
    goal = PoseStamped()

    # Define the goal pose (adjust these values accordingly)
    goal.header.frame_id = 'tianbot_mini/map' # change frame_id if the robot's name is not tianbot_mini
    goal.pose.position.x = x 
    goal.pose.position.y = y 
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 1 # default to 1 
    
    # Create a publisher for the /tianbot_mini/move_base_simple/goal topic
    pub = rospy.Publisher('/tianbot_mini/move_base_simple/goal', PoseStamped, queue_size=100)

    # Publish the goal 
    # sleep for a while and wait the publisher to pub 
    import time
    time.sleep(3)
    
    #publish the goal point 
    pub.publish(goal)
    print(goal)
    rospy.loginfo("Move base goal sent to /tianbot_mini/move_base_simple/goal")

def go_target(data:Bool):
    """go to the ball_recver when the uav find the goal

    Args:
        data (Bool): get from the "run_car", true for find else for none
    """
    if data.data :
        send_goal(-0.55, 0.02)

if __name__ == '__main__':
    try:
        send_goal() # go to the start position 
        rospy.Subscriber('/arrive_target', Bool, go_target)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")
