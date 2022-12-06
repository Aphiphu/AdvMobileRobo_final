#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def callback(msg):
    try:   
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig
        #resize image (half-size) for easier processing
        resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
        drawImg = resized
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        # test this
        blurFrame = cv2.GaussianBlur(grat, (blurRegion, blurRegion), 0)
        # changes ends here
        drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        edged = cv2.Canny(gray, 35, 125)
        cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        c = max(cnts, key = cv2.contourArea)

    except Exception as err:
    	print (err)
    showImage(drawImg)

def start_node():
    rospy.init_node('image_subscriber')
    rospy.loginfo('image subscriber node started')
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

