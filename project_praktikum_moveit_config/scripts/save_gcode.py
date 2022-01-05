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
        self.max_min_pos_service = rospy.Service("/show_max_min_pos", SetBool, self.callback_show_max_min_pos)
        self.gcode_point_service = rospy.Service("/save_gcode_point", SetBool, self.callback_gcode_point)
        self.save_gcode_point_service = rospy.Service("/save_all_gcode_points", SetBool, self.callback_save_gcode_point)
        rospy.on_shutdown(self.shutdown)
        self.limit = 0.1
        self.messages= []
        self.gcode = []
        self.gcode_point = []
        self.messages_str = []
        self.max_x = 0
        self.max_y = 0
        self.max_z = 0
        self.min_x = 0
        self.min_y = 0
        self.min_z = 0


    def callback_show_max_min_pos(self,req):
        if req.data:
            return True, f"Max_X:{self.max_x}, Min_X:{self.min_x}, Max_Y:{self.max_y}, Min_Y:{self.min_y}, Max_Z:{self.max_z}, Min_Z:{self.min_z}"

    def shutdown(self):
        print("\nShutting down:")
        try:
            print("Shutdown Successfuly")
        except Exception as e:
            print("ERROR: "+ str(e))

    def callback_gcode_point(self,req):
        self.gcode_point.append(self.gcode[-1])
        return True, f"point number {len(self.gcode_point)} was saved"

    def callback_save_gcode_point(self,req):
        if req.data:
            try:
                path = '/home/praktikant2/gcodes_examples/point_dumper.txt'
                mode = 'w' #just write and overwrite
                with open(path, mode) as f:
                    for line in self.gcode_point:
                        f.write(f'{line}')
                    #f.write(f'{self.gcode}')
            except Exception as e:
                print("ERROR", e)
            return True, f"GCODE File has been successfully created  and stored in {path}"
        return False, "GCODE File Failed"


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
                path = '/home/praktikant2/gcodes_examples/dumper.txt'
                mode = 'w' #just write and overwrite
                with open(path, mode) as f:
                    for line in self.gcode:
                        f.write(f'{line}')
                    #f.write(f'{self.gcode}')
            except Exception as e:
                print("ERROR", e)
            return True, f"GCODE File has been successfully created  and stored in {path}"
        return False, "GCODE File Failed"

    def callback_joints(self,msg):
        scale_mm = 1000
        positions = msg.position
        feed_ratio = 1000
        x_offset = 320
        y_offset = 660
        z_offset = -178
        y = round((scale_mm * positions[0]),2) #in mm
        x = round((scale_mm * positions[1]),2)
        z = round((scale_mm * positions[2]),2)
        b = round(math.degrees((positions[4])),2) #in deg
        c = round(math.degrees((positions[3])),2)

        if not self.messages:
            points = [x,y,z,b,c]
            coordinates_no_offset = [x-x_offset, y-y_offset, -z-z_offset]
            points_str = f"{x},{y},{z},{b},{c}\n"
            gcode_msg = f"G01X{coordinates_no_offset[0]}Y{coordinates_no_offset[1]}Z{coordinates_no_offset[2]}B{-b}C{360-c}F{feed_ratio}\n"
            self.max_x = coordinates_no_offset[0]
            self.max_y = coordinates_no_offset[1]
            self.max_z = coordinates_no_offset[2]
            self.min_x = coordinates_no_offset[0]
            self.min_y = coordinates_no_offset[1]
            self.min_z = coordinates_no_offset[2]
            self.messages.append(points)
            self.messages_str.append(points_str)
            self.gcode.append(gcode_msg)

        else:
            if abs(self.messages[-1][0] - x) > self.limit or abs(self.messages[-1][1] -y) > self.limit or abs(self.messages[-1][2] - z) > self.limit or abs(self.messages[-1][3] -b) > self.limit or abs(self.messages[-1][4] -c) > self.limit:
                points = [x,y,z,b,c]
                coordinates_no_offset = [x-x_offset, y-y_offset, -z-z_offset]
                points_str = f"{x},{y},{z},{b},{c}\n"
                gcode_msg = f"X{coordinates_no_offset[0]} Y{coordinates_no_offset[1]} Z{coordinates_no_offset[2]} B{-b} C{360-c} F{feed_ratio}\n"

                if coordinates_no_offset[0] > self.max_x:
                    self.max_x = coordinates_no_offset[0]
                if coordinates_no_offset[0] < self.min_x:
                    self.min_x = coordinates_no_offset[0]
                if coordinates_no_offset[1] > self.max_y:
                    self.max_y = coordinates_no_offset[1]
                if coordinates_no_offset[1] < self.min_y:
                    self.min_y = coordinates_no_offset[1]
                if coordinates_no_offset[2] > self.max_z:
                    self.max_z = coordinates_no_offset[2]
                if coordinates_no_offset[2] < self.min_z:
                    self.min_z = coordinates_no_offset[2]

                self.messages.append(points)
                self.messages_str.append(points_str)
                self.gcode.append(gcode_msg)


def main():
    SaveCode()
    rospy.init_node('save_code')
    rospy.loginfo("SAVE GCODE NODE IS LISTENING")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
