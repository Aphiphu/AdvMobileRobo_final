#!/usr/bin/env python
import rospy
import mediapipe
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
import cv2

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def callback(msg):
    try:   
        while true:
          bridge = CvBridge()
          orig = bridge.imgmsg_to_cv2(msg, "bgr8")
          drawImg = orig
          resized = cv2.resize(frame, (640, 480))
          hands, img = detector.findHands(resized)
          
          if hands:
            lmlist = hands[0]["lmlist"][:2]
            x1, y1 = lmlist[5]
            x2, y2 = lmlist[17]
            
            handdistance = math.sqrt((y2-y1) ** 2 + (x2-x1) ** 2)
            #handdistanceCM = int(coff[0] * handdistance **2 + coff[1] * handdistance + coff[2])
            
            if handtype = "R":
                distancetalker(handdistanceCM)
                continue
            else:
                distancetalker(handdistanceCM)
                continue
            
            
            
    except Exception as err:
    	print (err)
    
    showImage(drawImg)

def start_node():
    rospy.init_node('image_subscriber')
    rospy.loginfo('image subscriber node started')
    detector = HandDetector(detectionCon=0.8, maxHands=1)\
    
    rawlength = []
    reallength = []
    coff = np.polyfit(rawlength, reallength, 2)
    
    listener()

def listener():
    rospy.Subscriber("cv_camera/image_raw", Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def distancetalker(distance):
    pub = rospy.Publisher('distanceCM', Int, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
