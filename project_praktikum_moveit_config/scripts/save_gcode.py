#!/usr/bin/env python
import rospy
import serial
import time
import sys
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import math

class SaveCode():
    def __init__(self):
        self.save_joints_service = rospy.Service("/save_joints", SetBool, self.callback_save_joints)
        self.save_gcode_service = rospy.Service("/save_gcode", SetBool, self.callback_save_gcode)
        self.number_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback_joints)
        rospy.on_shutdown(self.shutdown)
        self.limit = 0.2
        self.messages= []
        self.gcode = []
        self.messages_str = []

    def shutdown(self):
        print("\nShutting down:")
        try:
            print("Shutdown Successfuly")
        except Exception as e:
            print("ERROR: "+ str(e))

    def callback_save_joints(self,req):
        if req.data:
            try:
                with open('Joints_positions.txt','w') as f:
                    for line in self.messages_str:
                        f.write(f'{line}')
                    #f.write(f'{self.messages_str}')
            except Exception as e:
                print("ERROR", e)
            return True, "JOINT File has been successfully created"
        return False, "JOINT File Failed"

    def callback_save_gcode(self,req):
        if req.data:
            try:
                with open('Gcode.txt','w') as f:
                    for line in self.gcode:
                        f.write(f'{line}')
                    #f.write(f'{self.gcode}')
            except Exception as e:
                print("ERROR", e)
            return True, "GCODE File has been successfully created"
        return False, "GCODE File Failed"

    def callback_joints(self,msg):
        scale_mm = 1000
        positions = msg.position
        y = round((scale_mm * positions[0]),2) #in mm
        x = round((scale_mm * positions[1]),2)
        z = round((scale_mm * positions[2]),2)
        b = round(math.degrees((positions[4])),2) #in deg
        c = round(math.degrees((positions[3])),2)

        if not self.messages:
            points = [x,y,z,b,c]
            points_str = f"{x},{y},{z},{b},{c}\n"
            gcode_msg = f"G01X{x}Y{y}Z{-z}B{b}C{c}F100\n"
            self.messages.append(points)
            self.messages_str.append(points_str)
            self.gcode.append(gcode_msg)

        else:
            if abs(self.messages[-1][0] - x) > self.limit or abs(self.messages[-1][1] -y) > self.limit or abs(self.messages[-1][2] - z) > self.limit or abs(self.messages[-1][3] -b) > self.limit or abs(self.messages[-1][4] -c) > self.limit:
                points = [x,y,z,b,c]
                points_str = f"{x},{y},{z},{b},{c}\n"
                gcode_msg = f"X{x} Y{y} Z{-z} B{b} C{c} F100\n"
                self.messages.append(points)
                self.messages_str.append(points_str)
                self.gcode.append(gcode_msg)


def main():
    SaveCode()
    rospy.init_node('save_code')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
