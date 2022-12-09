#!/usr/bin/env python
from std_msgs.msg import Int16
from std_msgs.msg import String
import rospy
import mediapipe
from cvzone.HandTrackingModule import HandDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2
import numpy as np
import math

goalarr = [0]

def callback1(data):
    print ("callback1 started")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
    distanceCM = data.data
    goalarr[0] = distanceCM
    
def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
    distanceCM = goalarr[0]
    curx = data.data 
    cury = data.data
    orient = data.data
    # function to convert the distanceCM to point (x, y) on the map
    # this requires the current postion and orientation of the robot
    # /odom topic has drift which when using map will be transformed for /map frame which is then transformed into the /base frame    
    
def distance2point(distanceCM, currentx, currenty, orientation):
    goalx = currentx + distanceCM
    goaly = currenty + distanceCM 
    goal = [goalx, goaly]  
    return goal
    
    
def listener():
    
    rospy.init_node('distancelistener', anonymous=True)
    print ("callback1")
    rospy.Subscriber('distanceCM', Int16, callback1)
    print ("callback2")
    rospy.Subscriber('base', String, callback2)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
