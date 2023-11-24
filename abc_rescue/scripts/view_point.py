#! /usr/bin/env python3
# data:2023年11月23日15:27:35
# auther:VegickenYe(Vegicken Ye)

import rospy
import time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32,Bool
import math
import sys

def qua2rpy(x,y,z,w):
    """
    Convert a quaternion (x, y, z, w) to Euler angles (Roll, Pitch, Yaw).

    Args:
    - x (float): The x component of the quaternion.
    - y (float): The y component of the quaternion.
    - z (float): The z component of the quaternion.
    - w (float): The w component of the quaternion.

    Returns:
    - roll (float): Roll angle in radians.
    - pitch (float): Pitch angle in radians.
    - yaw (float): Yaw angle in radians.
    """
    roll = math.atan2(2* (w * x +y * z), 1-2 *(x**2 + y**2))
    pitch = math.asin(2*(w*y -x*z))
    yaw = math.atan2(2*(w*z + x*y), 1-2*(z**2 + y **2)) 
    return roll,pitch, yaw

def make_cmd_vel(x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
    """
    Create a geometry_msgs/Twist message with specified linear and angular velocities.

    Parameters:
    - x (float): Linear velocity along the x-axis.
    - y (float): Linear velocity along the y-axis.
    - z (float): Linear velocity along the z-axis.
    - roll (float): Angular velocity around the x-axis (roll).
    - pitch (float): Angular velocity around the y-axis (pitch).
    - yaw (float): Angular velocity around the z-axis (yaw).

    Returns:
    - geometry_msgs.msg.Twist: Twist message with specified velocities.
    """
    global vel
    msg = Twist()
    
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z

    msg.angular.x = roll
    msg.angular.y = pitch
    msg.angular.z = yaw

    return msg

pose_x, pose_y,pose_z, ori_r,ori_p,ori_y, pose_z =None,None,None,None,None,None, None

def update_pose(pose:PoseStamped):
    """
    Update global pose variables based on the received PoseStamped message.

    Parameters:
    - pose (geometry_msgs.msg.PoseStamped): PoseStamped message containing pose information.

    Global Variables Updated:
    - global pose_x (float): X-coordinate of the pose.
    - global pose_y (float): Y-coordinate of the pose.
    - global pose_z (float): Z-coordinate of the pose.
    - global ori_r (float): Roll angle of the orientation.
    - global ori_p (float): Pitch angle of the orientation.
    - global ori_y (float): Yaw angle of the orientation.
    """
    global pose_x, pose_y, pose_z,ori_r,ori_p,ori_y
    pose_x = pose.pose.position.x
    pose_y = pose.pose.position.y
    pose_z = pose.pose.position.z
    xx = pose.pose.orientation.x
    yy = pose.pose.orientation.y
    zz = pose.pose.orientation.z
    ww = pose.pose.orientation.w
    ori_r,ori_p,ori_y = qua2rpy(xx, yy, zz, ww)
    #print(pose_x,pose_y,pose_z, ori_r, ori_p, ori_y)


target_x, target_y = 0,0
target_z = 0.8

def compute_dis(x1,y1, x2,y2):
    """
    Compute the Euclidean distance between two points.

    Parameters:
    - x1 (float): X-coordinate of the first point.
    - y1 (float): Y-coordinate of the first point.
    - x2 (float): X-coordinate of the second point.
    - y2 (float): Y-coordinate of the second point.

    Returns:
    - float: Euclidean distance between the two points.
    """
    return math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2)

discount = 3 # Calculate the rotational velocity based on the difference between angles

def move_to_target_location(boundary):
    """
    Move UAV to a target location until it reaches a specified distance threshold.

    Parameters:
    - boundary (float): The distance threshold to the target location.

    Returns:
    - bool: True if UAV successfully find the ball receiver, False otherwise.
    """
    global target_x,target_y, target_z
    global pose_x, pose_y,discount, pose_z
    global find_tag
    global third_quadrant
    dis =compute_dis(target_x, target_y, pose_x, pose_y)
    max_vel =0.15
    print("now go to")
    while dis>boundary:
        if find_tag and third_quadrant:
            b_msg = Bool()
            b_msg.data = True
            fin_pub.publish(b_msg)
            print('find the ball receiver and end the view_point file')
            return True
            sys.exit()
            break
        else:
            vel_x = dis /discount
            vel_x = (vel_x / abs(vel_x)) * (max_vel if abs(vel_x) >=max_vel else abs(vel_x))
            msg = make_cmd_vel(x=vel_x)
            cmd_pub.publish(msg)
            dis =compute_dis(target_x, target_y, pose_x, pose_y)
    print('have arrived at', pose_x,pose_y)
    msg = make_cmd_vel()
    cmd_pub.publish(msg)
    return False

def rotate_to_target_angle(angle:float):
    """
    Rotate UAV to a specified angle.

    Parameters:
    - angle (float): The target angle in radians.

    Returns:
    - bool: True if UAV successfully find the ball receiver, False otherwise.
    """
    global ori_y
    global find_tag
    global third_quadrant
    print('rotate to target angle', angle, ori_y)
    angle = angle 
    if angle < -math.pi:
        angle+= 2*math.pi
    elif angle>=math.pi :
        angle -= 2*math.pi
    if abs(abs(angle) -math.pi) <= 6/180 * math.pi:
        angle = 0
    # assert angle - ori_y>= -math.pi and angle - ori_y<= math.pi
    threhold = 0.2

    print('residual is ', angle-ori_y)
    
    while abs(angle - ori_y) > (6/180 * 3.14):
        if find_tag and third_quadrant:
            b_msg = Bool()
            b_msg.data = True
            fin_pub.publish(b_msg)
            print('find the ball receiver and end the view_point file')
            return True
            sys.exit()
            # time.sleep(10000)
            break
        else:
            # global ori_y
            vel_y = (angle - ori_y) /discount
            vel_y = (vel_y / abs(vel_y)) * (threhold if abs(vel_y) > threhold else abs(vel_y))
            # print('residual ', angle-ori_y, 'vel_y  ', vel_y)
            msg = make_cmd_vel(yaw=vel_y)
            cmd_pub.publish(msg)
            # time.sleep(0.2)
    print('now oritation is ', ori_y)
    stop = make_cmd_vel()
    cmd_pub.publish(stop)
    # time.sleep(10)
    # fin = Bool()
    # fin.data=True
    # fin_pub.publish(fin)
    return False

def update_angle():
    """
    Update the target angle based on the current UAV position and target coordinates.

    Returns:
    - float: The updated target angle in radians.
    """
    global target_x, target_y, pose_x, pose_y
    dy = target_y - pose_y
    dx = target_x - pose_x

    target_angle = math.atan2(dy, dx)
    return target_angle

def update_target(x,y):
    """
    Update the target coordinates and navigate the UAV to the specified location.

    Parameters:
    - x (float): The X-coordinate of the target location.
    - y (float): The Y-coordinate of the target location.

    Returns:
    - None
    """
    global target_x, target_y, pose_x, pose_y
    msg = make_cmd_vel()
    cmd_pub.publish(msg)
    target_x = x
    target_y = y
    distance =compute_dis(target_x, target_y, pose_x, pose_y)
    print("distance is ",distance)
    for i in  [0.6*distance, 0.2*distance]:
        target_angle = update_angle()
        rotate_to_target_angle(target_angle)
        ret = move_to_target_location(i)
        print("distance is ",i)
        if ret : 
            print('find the ball receiver and end the view_point file')
            break
        
    print('finished view point ', x, y)

target_coords = [(1,-1), (0,0), (-1,1),(-1, -1)]
target_coords_2 = [(-0.7,-0.7), (-1,0), (0,0),(0,-1),(-1, -1)]

def start(start:Bool):
    """
    Start the UAV navigation based on the received start signal.

    Parameters:
    - start_signal (Bool): The start signal to initiate the UAV navigation.

    Returns:
    - None
    """
    global target_coords
    global target_coords_2
    global third_quadrant
    third_quadrant = False
    msg = Bool()
    msg.data = False
    fin_pub.publish(msg)
    cmd_msg = make_cmd_vel()
    cmd_pub.publish(cmd_msg)
    time.sleep(2)
    if start.data == True:
        print('start')
        for x,y in target_coords:
            update_target(x, y)
        print("finish 4 points")
        third_quadrant = True
        for x,y in target_coords_2:
            update_target(x, y)
    
find_tag = False

def find_target(tag:Bool):
    """
    Update the flag indicating whether the UAV has detected the ball receiver based on the received signal.

    Parameters:
    - tag_detected (Bool): The signal indicating whether the ball receiver is detected.

    Returns:
    - None
    """
    global find_tag
    if tag.data == True:
        find_tag = True
        #print("find target and subscribe") 
    else:
        find_tag = False



if __name__ =='__main__':
    try:
        rospy.init_node('turn_angle')
        rospy.Subscriber("/pose", PoseStamped, callback=update_pose)
        rospy.Subscriber("/start_go", Bool, callback=start)
        rospy.Subscriber("/find_tag", Bool, callback=find_target)
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        fin_pub = rospy.Publisher("/view_fin", Bool, queue_size=10 )
        msg = make_cmd_vel()
        cmd_pub.publish(msg)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
