#!/usr/bin/env python
import rospy

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
    distanceCM = data.data
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('distanceCM', Int, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
