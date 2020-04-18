#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from avse_common.msg import NavSatFix

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/android/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
