#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class ListenerPositions:
    def __init__(self):
        self.pub = rospy.Publisher("/chatter", String, queue_size=10)
        #self.rate = rospy.Rate(100)
        self.number_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback)
    def callback(self, msg):
        #scale_mm = 1000
        #self.counter += msg.data
        #new_msg = Int64()
        #new_msg.data = self.counter
        #round((scale_mm * positions[1]), 1)
        #math.floor((scale_mm * positions[1]))
        positions = msg.position
        a = round(math.degrees(positions[0]),2) #in mm
        b = round(math.degrees(positions[1]),2)
        c = round(math.degrees(positions[2]),2)
        d = round(math.degrees(positions[3]),2) #in deg
        e = round(math.degrees(positions[4]),2)
        h = round(math.degrees(positions[5]),2)

        new_msg = f"G01 X{a} Y{b} Z{c} A{d} B{e} C{h}"
        #print(new_msg)
        #self.rate.sleep()
        self.pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('listener_positions')
    ListenerPositions()
    rospy.spin()
