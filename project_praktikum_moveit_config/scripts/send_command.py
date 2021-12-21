#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        cmd = str(input("ENTER COMMAND: "))
        rospy.loginfo("SENDING: " + cmd)
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
