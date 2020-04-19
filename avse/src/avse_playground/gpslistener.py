#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def callback(data):
    global gps_lat, gps_long
    rospy.loginfo(data.latitude)
    gps_lat = data.latitude
    print(gps_lat)


def listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/android/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
