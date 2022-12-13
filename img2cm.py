#!/usr/bin/env python
from std_msgs.msg import Int16
import rospy
import mediapipe
from cvzone.HandTrackingModule import HandDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2
import numpy as np
import math

def palmopen(hands):
    lmlist = hands["lmlist"]

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def callback(msg):
    try:   
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig
        resized = cv2.resize(orig, (640, 480))
        detector = HandDetector(detectionCon=0.7, maxHands=1)
        hands, img = detector.findHands(resized)
        rawlength = [245, 170, 150, 120, 70, 50]
        reallength = [25, 30, 35, 40, 45, 100]
        coff = np.polyfit(rawlength, reallength, 2)
          
        if hands:
           #check if its an open palm function insert here
           #palmopen(hands[0])
 
           points = hands[0]["lmList"]
           x1, y1 = points[5][:2]
           x2, y2 = points[17][:2]
           
          
           handdistance = math.sqrt((y2-y1) ** 2 + (x2-x1) ** 2)
           

         
           handdistanceCM = int(coff[0] * handdistance **2 + coff[1] * handdistance + coff[2])
           print (handdistanceCM) 
           distancetalker(handdistanceCM)
           
            
    except Exception as err:
    	print (err)
    
    showImage(drawImg)

def start_node():
    rospy.init_node('imagesubscriber')
    rospy.loginfo('image subscriber node started')

    listener()

def listener():
    rospy.Subscriber("cv_camera/image_raw", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def distancetalker(distance):
    pub = rospy.Publisher('distanceCM', Int16, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()
       

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
