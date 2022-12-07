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
      with handsModule.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=2) as hands:
        while true:
          bridge = CvBridge()
          orig = bridge.imgmsg_to_cv2(msg, "bgr8")
          drawImg = orig
          resized = cv2.resize(frame, (640, 480))
          results = hands.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))
          if results.multi_hand_landmarks != None:
                for handLandmarks in results.multi_hand_landmarks:
                    drawingModule.draw_landmarks(frame1, handLandmarks, handsModule.HAND_CONNECTIONS)
                    for point in handsModule.HandLandmark:
                      
                      normalizedLandmark = handLandmarks.landmark[point]
                      pixelCoordinatesLandmark= drawingModule._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, 640, 480)
                      
                      #point 0 is the wrist
                      if point == 0:
                          print(point)
                          print(pixelCoordinatesLandmark)
                          print(normalizedLandmark)
           

    except Exception as err:
    	print (err)
    
    showImage(drawImg)

def start_node():
    rospy.init_node('image_subscriber')
    rospy.loginfo('image subscriber node started')
    drawingModule = mediapipe.solutions.drawing_utils
    handsModule = mediapipe.solutions.hands
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
