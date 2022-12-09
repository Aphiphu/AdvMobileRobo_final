#!/usr/bin/env python
import rospy
import mediapipe
from cvzone.HandTrackingModule import HandDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2
import numpy as np
import math

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def callback(msg):
    
          try:   
          
              bridge = CvBridge()
              orig = bridge.imgmsg_to_cv2(msg, "bgr8")
              drawImg = orig
              resized = cv2.resize(orig, (640, 480))
              detector = HandDetector(detectionCon=0.8, maxHands=1) 
              hands, img = detector.findHands(resized)
             
          
              if hands: 
                 
                 rospy.loginfo("Detected")
    
                 print(hands[0]["lmList"])
                 points = hands[0]["lmList"]
                 x1, y1 = points[5][:2]
                 x2, y2 = points[17][:2]
            
                 handdistance = math.sqrt((y2-y1) ** 2 + (x2-x1) ** 2)
                 print ("Handdistance currently is ", handdistance)

            
          except Exception as err:
    	      print (err)
    
          showImage(drawImg)

def start_node():
    rospy.init_node('imagesubscriber')
    rospy.loginfo('image subscriber node started')
    
    rawlength =  []
    reallength = [10, 15, 20, 25, 30, 35, 40, 45, 50]
   # coff = np.polyfit(rawlength, reallength, 2)
    
    listener()

def listener():
    rospy.Subscriber("cv_camera/image_raw", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
       

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
