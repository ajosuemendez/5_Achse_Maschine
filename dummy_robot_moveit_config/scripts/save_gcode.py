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
        #self.save_joints_service = rospy.Service("/save_joints", SetBool, self.callback_save_joints)
        #self.save_gcode_service = rospy.Service("/save_gcode", SetBool, self.callback_save_gcode)
        self.number_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback_joints)
        self.get_gcode_service = rospy.Service("/get_gcode", SetBool, self.get_gcode)
        self.start_recording_subscriber = rospy.Service("/start_record", SetBool, self.start_record)
        self.clear_gcode_buffer_subscriber = rospy.Service("/clear_gcode_buffer", SetBool, self.clear_gcode_buffer)
        #self.gcode_point_service = rospy.Service("/save_gcode_point", SetBool, self.callback_gcode_point)
        #self.save_gcode_point_service = rospy.Service("/save_all_gcode_points", SetBool, self.callback_save_gcode_point)
        rospy.on_shutdown(self.shutdown)
        self.limit = 1
        self.messages= []
        self.gcode = ""
        self.gcode_point = []
        self.messages_str = []
        self.feed_ratio = 1000
        self.start_record = False

    def shutdown(self):
        print("\nShutting down:")
        try:
            print("Shutdown Successfuly")
        except Exception as e:
            print("ERROR: "+ str(e))

    # def callback_gcode_point(self,req):
    #     self.gcode_point.append(self.gcode[-1])
    #     return True, f"point number {len(self.gcode_point)} was saved"
    #
    # def callback_save_gcode_point(self,req):
    #     if req.data:
    #         try:
    #             path = '/home/praktikant2/gcodes_examples/point_dumper.txt'
    #             mode = 'w' #just write and overwrite
    #             with open(path, mode) as f:
    #                 for line in self.gcode_point:
    #                     f.write(f'{line}')
    #                 #f.write(f'{self.gcode}')
    #         except Exception as e:
    #             print("ERROR", e)
    #         return True, f"GCODE File has been successfully created  and stored in {path}"
    #     return False, "GCODE File Failed"
    #
    #
    # def callback_save_joints(self,req):
    #     if req.data:
    #         try:
    #             with open('Joints_positions.txt','w') as f:
    #                 for line in self.messages_str:
    #                     f.write(f'{line}')
    #                 #f.write(f'{self.messages_str}')
    #         except Exception as e:
    #             print("ERROR", e)
    #         return True, "JOINT File has been successfully created"
    #     return False, "JOINT File Failed"
    #
    # def callback_save_gcode(self,req):
    #     if req.data:
    #         try:
    #             path = '/home/praktikant2/gcodes_examples/dumper.txt'
    #             mode = 'w' #just write and overwrite
    #             with open(path, mode) as f:
    #                 for line in self.gcode:
    #                     f.write(f'{line}')
    #                 #f.write(f'{self.gcode}')
    #         except Exception as e:
    #             print("ERROR", e)
    #         return True, f"GCODE File has been successfully created  and stored in {path}"
    #     return False, "GCODE File Failed"

    def start_record(self,req):
        if req.data:
            try:
                self.start_record = True
                return True, "Starting recording gcode"
            except Exception as e:
                print(e)
                return False, "Unable to start recording gcode"

    def clear_gcode_buffer(self, req):
        if req.data:
            try:
                self.gcode = ""
                return True, f"Current Gcode Buffer: {self.gcode}"
            except Exception as e:
                print(e)
                return False, "Unable to Clear the Gcode Buffer"


    def callback_joints(self,msg):
        if self.start_record:
            positions = msg.position #array containing all the joint values
            a = round(math.degrees(positions[0]),3)
            b = round(math.degrees(positions[1]),3)
            c = round(math.degrees(positions[2]),3)
            d = round(math.degrees(positions[3]),3)
            e = round(math.degrees(positions[4]),3)
            h = round(math.degrees(positions[5]),3)

            if not self.messages:
                points = [a,b,c,d,e,h]
                points_str = f"{a},{b},{c},{d},{e},{h}\n"
                gcode_msg = f"G01 A{a} B{b} C{c} D{d} E{e} H{h} F{self.feed_ratio}\n"
                self.messages.append(points)
                self.messages_str.append(points_str)
                self.gcode += gcode_msg

            else:
                if abs(self.messages[-1][0] - a) > self.limit or abs(self.messages[-1][1] -b) > self.limit or abs(self.messages[-1][2] - c) > self.limit or abs(self.messages[-1][3] -d) > self.limit or abs(self.messages[-1][4] -e) > self.limit or abs(self.messages[-1][5] -h) > self.limit:
                    points = [a,b,c,d,e,h]
                    points_str = f"{a},{b},{c},{d},{e},{h}\n"
                    gcode_msg = f"G01 A{a} B{b} C{c} D{d} E{e} H{h} F{self.feed_ratio}\n"

                    self.messages.append(points)
                    self.messages_str.append(points_str)
                    self.gcode += gcode_msg

    def get_gcode(self,req):
        if req.data:
            try:
                self.start_record = False
                return True, self.gcode
            except Exception as e:
                print(e)
                return False,"no Gcode was generated"



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
