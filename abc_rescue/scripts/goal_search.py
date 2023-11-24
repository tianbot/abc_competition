#! /usr/bin/env python
# date: 2023年11月23日15:18:19
# author: Lr_2002(Xianhao Wang)

import numpy as np
import cv2 as cv
import rospy
import math
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool,Empty, String
from geometry_msgs.msg import Twist, PoseStamped


upside_goal_cnt = 0 # record how many times the goal has been found
start_go_point = False # Determine whether start go to the destination
pose_x, pose_y,pose_z, ori_r,ori_p,ori_y, pose_z =None,None,None,None,None,None, None # basic info of the uav

def update_pose(pose:PoseStamped):
    """update the position

    Args:
        pose (PoseStamped): where the uav is now 
    """
    global pose_x, pose_y, pose_z

    pose_x = pose.pose.position.x
    pose_y = pose.pose.position.y
    pose_z = pose.pose.position.z


def make_cmd_vel(x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
    """make cmd vel

    Args:
        x (int, optional): x. Defaults to 0.
        y (int, optional): y. Defaults to 0.
        z (int, optional): z. Defaults to 0.
        roll (int, optional): roll. Defaults to 0.
        pitch (int, optional): pitch. Defaults to 0.
        yaw (int, optional): yaw. Defaults to 0.

    Returns:
        msg (Twist) : the cmd_vel
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

def go_to_the_ball_recver(x, y, w, h):
    """use image to go to the ball_recver

    Args:
        x (int): the x of the target
        y (int): the y of the target
        w (int): the width of the image
        h (int): the height of the image

    Returns:
        (vel_x, vel_y): the velocity of the x and y 
    """
    global upside_goal_cnt 
    cx = w // 2  # center x 
    cy =  h//2-0.05*h # 0.05 for the bias of the camera
    vel_x = 0
    vel_y = 0
    bnd_threshold = 8 # boundary threshold check of the ball_recver position
    pixel_dis = math.sqrt( (x-cx)**2 + (y-cy) **2 ) # the pix_dis from the image
    base_v = pixel_dis / 100 # base_v (the threshold of velocity)
    if base_v != 0:
        thd = 0.1 if base_v > 100 else 0.05
        base_v = (base_v/abs(base_v)) * (thd if abs(base_v) > thd else abs(base_v))   
    if x > cx + bnd_threshold:
        vel_y = -base_v
    if x < cx - bnd_threshold: 
        vel_y = base_v
    if y > cy + bnd_threshold:
        vel_x = -base_v
    if y < cy - bnd_threshold:
        vel_x = base_v
    global start_go_point, pose_z
    if start_go_point:
        if x < cx +bnd_threshold and x > cx -bnd_threshold and y>cy-bnd_threshold and y < cy+bnd_threshold:
            upside_goal_cnt +=1 
            if upside_goal_cnt >= 20:
                # todo the ball might not go into the recver
                throw_ball = String()
                throw_ball.data= 'EXT servo 120' # release the ball
                throw_pub.publish(throw_ball)
                print('Throw the Ball')
                return (vel_x, vel_y)
        else:
            if upside_goal_cnt != 0:
                upside_goal_cnt = 0 
        if x < cx +25 and x > cx -25 and y >cy-25 and y < cy+25:
            if(vel_x>=vel_y):
                msg = make_cmd_vel(x=vel_x, y=0, z=0, yaw=0)
                cmd_pub.publish(msg)
            else:
                msg = make_cmd_vel(x=0, y=vel_y, z=0, yaw=0)
                cmd_pub.publish(msg)
        else:
            msg = make_cmd_vel(x=vel_x, y=vel_y, z=0, yaw=0)
            cmd_pub.publish(msg)
    return (vel_x, vel_y)

def find_biggest_rect(rect_list):
    """find the biggest rectangle
    todo: 误识别
    Args:
        rec_list (list): list from all the rect 

    Returns:
        (x1,y1), (x2,y2): two point of the rectangle 
    """
    if len(rect_list) == 0 :
        return None, None
    x11, y11 = 10000000,1000000
    x22, y22 = -1000,-1000
    for x1, y1, x2, y2 in rect_list:
        x11 = min(x11, x1)
        y11 = min(y11, y1)
        x22 = max(x22, x2)
        y22 = max(y22, y2)
    return (x11, y11),(x22, y22)

find_goal_cnt=0 

def process_image(img):
    """read img, hsv process and give cmd_vel

    Args:
        img (np.array): image 
    """
    # init the detecting windows
    targetPos_x = 0
    targetPos_y = 0
    lastPos_x = 0
    lastPos_y = 0

    # make a black window
    cv.namedWindow("image", 0)
    cv.resizeWindow("image", 1000, 750)
    cv.namedWindow('image')

    w, h, c = img.shape
    if w == 240:
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)   # rotate the image 
    # transfer from bgr to hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    # get now position
    H_low = 0
    H_high = 179
    S_low = 0
    S_high = 255
    V_low = 0
    V_high = 7
    threshold= 800

    # HSV value
    lower_color = np.array([H_low, S_low, V_low])
    high_color = np.array([H_high, S_high, V_high])
    # pick hsv color
    mask = cv.inRange(hsv, lower_color, high_color)  # get mask

    # add the img and mask
    res = cv.bitwise_and(img, img, mask=mask)
    # find contours
    # CV_RETR_EXTERNAL detect the outside boundary
    # CV_CHAIN_APPROX_NONE save all the 
    contours,_ = cv.findContours(mask, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)
    # drawcontours in the image
    cv.drawContours(res, contours, -1, (0, 255, 0), 3)
    # draw rectangle
    x, y, w, h = 0, 0, 0, 0
    rec_list = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > threshold:  # filter the threshold
            x, y, w, h = cv.boundingRect(cnt)
            lastPos_x = targetPos_x
            lastPos_y = targetPos_y
            targetPos_x = int(x + w / 2)
            targetPos_y = int(y + h / 2)
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
            rec_list.append((x,y,x+w,y+h))

    rec = find_biggest_rect(rec_list)
    global start_go_point,find_goal_cnt
    if rec[0] is not None:
        find_goal_cnt=find_goal_cnt+1
        if (find_goal_cnt>=50):
            msg = Bool()
            msg.data = True
            find_target_pub.publish(msg)
        cv.rectangle(img, rec[0], rec[1], (0,0, 255), 3)
        targetPos_x = int(rec[0][0] / 2 + rec[1][0]  /2 )
        targetPos_y = int(rec[0][1] / 2 + rec[1][1]  /2 )
        cv.circle(img, (targetPos_x, targetPos_y), 10, (255, 0, 0), 2) # 绘制方框中心
        cv.line(img, (int(img.shape[1] //2 ),int(img.shape[0] // 2-0.05*img.shape[0])), (targetPos_x, targetPos_y), (0,0,255))
        if start_go_point:
            vell  = go_to_the_ball_recver(targetPos_x, targetPos_y, img.shape[1], img.shape[0])
    else:
        # pub true if find the ball_recver
        msg = Bool()
        msg.data = False
        find_goal_cnt = 0
        find_target_pub.publish(msg)

    imgs = np.hstack([img, res])  # concate two images
    cv.imshow('image', imgs)
    cv.waitKey(1)




def call_image(msg:Image):
    """read image from buffer and send it to cv

    Args:
        msg (Image): image from "image_raw"
    """
    data = msg.data
    tmp = np.frombuffer(data[:230400],dtype=np.uint8)
    tmp= tmp.reshape((240, 320, 3))
    process_image(tmp)



global one_stop
one_stop = True
def  finish_view(msg):
    flag = msg.data
    if flag == True:
        global start_go_point
        global one_stop
        start_go_point = True
        #发送小车启动
        a_msg = Bool()
        a_msg.data = True
        arrive_pub.publish(a_msg)
        #完成寻点,停止无人机
        if one_stop:
            b_msg = make_cmd_vel()
            for i in range(3):
                time.sleep(1)
                print("---------------stop the uav-------------")
                cmd_pub.publish(b_msg)
            b_msg = make_cmd_vel(z=-0.6)
            cmd_pub.publish(b_msg)
            one_stop=False
       
    else:
        start_go_point = False
        msg = Bool()
        msg.data = False
        arrive_pub.publish(msg)

if __name__=='__main__':
    try:
        rospy.init_node('goal_search')
        rospy.Subscriber("image_raw", Image, callback=call_image)
        rospy.Subscriber("/view_fin", Bool, callback=finish_view)
        rospy.Subscriber("/pose", PoseStamped, callback=update_pose)
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # land_pub = rospy.Publisher('/land', Empty, queue_size=10)
        throw_pub = rospy.Publisher('/sdk_cmd', String, queue_size=10)
        arrive_pub = rospy.Publisher("/arrive_target",Bool,queue_size=10)
        find_target_pub = rospy.Publisher("/find_tag",Bool,queue_size=10)
        rospy.spin()
    
    except rospy.ROSInternalException:
        pass