#!/usr/bin/env python
import rospy
import serial
import time
import sys
import re
from std_msgs.msg import String

class ExchangeData():
    def __init__(self):
        self.pub = rospy.Publisher("/receive_cmd", String, queue_size=1)
        self.sub = rospy.Subscriber("/send_cmd", String, self.callback)

    def callback(self, msg):
        str_cmd = str(msg)
        x = re.serach
        str_cmd[index:]
        print(str_cmd)
        self.pub.publish(str_cmd)

def main():
    ExchangeData()
    rospy.init_node('exchange_data')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
