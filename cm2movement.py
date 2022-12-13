#!/usr/bin/env python
from std_msgs.msg import Int16
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import rospy
import mediapipe
from cvzone.HandTrackingModule import HandDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2
import numpy as np
import math



def callback1(data):
    print ("callback1 started")
    distanceCM = data.data
    
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = (distanceCM/100.0)
    goal.pose.position.y = 0.0 
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.sleep(1)
    print(goal)
    
    goal_publisher.publish(goal)
    
    
       
def listener():
    
    rospy.init_node('distancelistener', anonymous=True)
    print ("callback")
    rospy.Subscriber('distanceCM', Int16, callback1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
