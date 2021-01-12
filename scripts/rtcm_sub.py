#!/usr/bin/python


import rospy
from mavros_msgs.msg import RTCM
from ntrip_ros.msg import RTCM

from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def rtcm_cb(msg):
    print 'in callback'
    print msg


if __name__ == '__main__':
    rospy.init_node('rtcm_sub', anonymous=True)
    print 'creating callback'
    rospy.Subscriber("rtcm", RTCM, rtcm_cb)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()
