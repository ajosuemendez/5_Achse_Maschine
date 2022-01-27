#!/usr/bin/env python
import rospy
import serial
import actionlib
import time
import sys
from std_msgs.msg import String
from project_praktikum_moveit_config.srv import SendCommand,SendCommandResponse
from project_praktikum_moveit_config.msg import SerialCommunicationAction, SerialCommunicationFeedback, SerialCommunicationResult

RX_BUFFER_SIZE = 128

class SerialCom():
    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.ser = self.init_ser()
        self.serviceCommand = rospy.Service("/cmd_input", SendCommand, self.cmdCallback)
        self.action_server = actionlib.SimpleActionServer("serial_com", SerialCommunicationAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
        rospy.on_shutdown(self.shutdown)

    def execute_cb(self, goal):
        success = True
        feedback_action = SerialCommunicationFeedback()
        result_action = SerialCommunicationResult()
        rate = rospy.Rate(100)

        try:
            if self.ser.isOpen():
                try:
                    print ("OPPENING PORT: " + self.port)
                    self.ser.flushInput()
                    ########DENIS implementation#######
                    # c_line = []
                    # lines = goal.command.split("\n")
                    # i=0
                    # while i < len(lines):
                    #     if self.action_server.is_preempt_requested():
                    #         success = False
                    #         break
                    #
                    #
                    #     l_block = lines[i].strip()
                    #     # Track number of characters in grbl serial read buffer
                    #     grbl_out = ''
                    #     #time.sleep(0.1)
                    #     while self.ser.in_waiting>0:
                    #         #print(self.ser.in_waiting)
                    #         out_temp = self.ser.readline().decode('utf-8') # Wait for grbl response
                    #         rate.sleep()
                    #         if out_temp.find('ok') >= 0 or out_temp.find('error') >= 0:
                    #             grbl_out += out_temp;
                    #             grbl_out += f"Gcode Line: {i+1}"; # Add line finished indicator
                    #             del c_line[0] # Delete the block character count corresponding to the last 'ok'
                    #
                    #         else:
                    #             print("DEBUG:", out_temp)
                    #
                    #
                    #     #print(f"BItch: {sum(c_line) + len(l_block)}")
                    #     if sum(c_line) + len(l_block) < RX_BUFFER_SIZE-1:
                    #         print(f"SND: {i} :  {l_block}")
                    #         self.ser.write(str.encode(f'{l_block}\n')) # Send g-code block to grbl
                    #         rate.sleep()
                    #         i+=1
                    #         c_line.append(len(l_block)+1)
                    #         feedback_action.feedback = f"BUF:{str(sum(c_line))} REC:{grbl_out}"
                    #         print(feedback_action.feedback)
                    #         result_action.result += feedback_action.feedback + "\n"
                    #         self.action_server.publish_feedback(feedback_action)
                    #
                    #     else:
                    #         rate.sleep()
                    ########END OF DENIS IMPLEMENTATION#########
                    ########START FROM OLD IMPLEMENTATION#######
                    l_count = 0
                    g_count = 0
                    c_line = []
                    lines = goal.command.split("\n")
                    # periodic() # Start status report periodic timer
                    for line in lines:
                        if self.action_server.is_preempt_requested():
                            success = False
                            break
                        l_count += 1 # Iterate line counter
                        l_block = line.strip()
                        c_line.append(len(l_block)+1) # Track number of characters in grbl serial read buffer
                        grbl_out = ''
                        while sum(c_line) >= RX_BUFFER_SIZE-1 | self.ser.inWaiting():
                            out_temp = self.ser.readline().decode('utf-8')# Wait for grbl response
                            rate.sleep()
                            if out_temp.find('ok') < 0 and out_temp.find('error') < 0:
                                print("  Debug: ",out_temp) # Debug response
                            else:
                                grbl_out += out_temp
                                g_count += 1 # Iterate g-code counter
                                grbl_out += "Gcode Line:" + str(g_count) # Add line finished indicator
                                del c_line[0] # Delete the block character count corresponding to the last 'ok'

                        self.ser.write(str.encode(f'{l_block}\n')) # Send g-code block to grbl
                        rate.sleep()
                        feedback_action.feedback = f"BUF:{str(sum(c_line))} REC:{grbl_out}"
                        result_action.result += feedback_action.feedback + "\n"
                        self.action_server.publish_feedback(feedback_action)

                    ####END OF OLOD IMPLEMENTATION#######

                    if success:
                        self.action_server.set_succeeded(result_action)

                    print("G-code streaming finished!\n")
                    print("WARNING: Wait until grbl completes buffered g-code blocks before exiting.")

                except Exception as e:
                    print("ERROR: "+ str(e))
        except Exception as e:
            print("ERROR: "+ str(e))

    def init_ser(self):
        return serial.serial_for_url(self.port, self.baudrate)

    def shutdown(self):
        print("\nCLOSING PORT: " + self.port)
        try:
            self.ser.close()
        except Exception as e:
            print("ERROR: "+ str(e))

    def cmdCallback(self, msg):
        #if not self.ser.isOpen():
        #    try:
        #        self.ser.open()
        #    except Exception as e:
        #        print("Error open serial port:" + str(e))
        ######Simple Stream######
        # try:
        #     if self.ser.isOpen():
        #         try:
        #             print ("OPPENING PORT: " + self.port)
        #             #self.ser.flushInput()
        #             #self.ser.flushOutput()
        #             #self.rec_msg = msg.data
        #             self.rec_msg = msg.command #Uncomment this line later
        #             if self.rec_msg:
        #                 if self.rec_msg =="settings":
        #                     self.rec_msg = "$$"
        #                 print(f'SENDING:{self.rec_msg}')
        #                 bytes = self.ser.write(str.encode(f'{self.rec_msg}\r\n'))
        #                 print(f"BYTES SENT:{bytes}")
        #                 time.sleep(0.1)
        #                 bytesToRead = self.ser.inWaiting()
        #                 print("BYTES RECEIVED: ", bytesToRead)
        #                 data_raw = self.ser.read(bytesToRead).decode('utf-8')
        #                 str_data_raw = str(data_raw)
        #                 return SendCommandResponse(str_data_raw, True) #uncomment this line later
        #                 split_data = str_data_raw.split('\\r\\n')#comment the rest and return str_data_raw
        #                 print("RECEIVED [INFO: %s]: " % rospy.get_time())
        #                 for i in split_data:
        #                     print(i)
        #             #print("CLOSING PORT")
        #             #self.ser.close()
        #         except Exception as e1:
        #             print("error communicating....:" +str(e1))
        # except Exception as e:
        #     print("COULDNT OPEN PORT, ERROR: "+ str(e))
        ######Compleex Stream######
        try:
            if self.ser.isOpen():
                try:
                    print ("OPPENING PORT: " + self.port)
                    #self.ser.flushInput()
                    #self.ser.flushOutput()
                    #self.rec_msg = msg.data
                    self.rec_msg = msg.command
                    if self.rec_msg:
                        if not msg.simple_stream:
                            return_msg = ""
                            l_count = 0
                            g_count = 0
                            c_line = []
                            lines = self.rec_msg.split("\n")
                            for line in lines:
                                l_count += 1 # Iterate line counter
                                l_block = line.strip()
                                c_line.append(len(l_block)+1)# Track number of characters in grbl serial read buffer
                                grbl_out = ''
                                #time.sleep(0.1)
                                while sum(c_line) >= RX_BUFFER_SIZE-1 | self.ser.inWaiting():
                                    out_temp = self.ser.readline().decode('utf-8') # Wait for grbl response
                                    #time.sleep(0.1)
                                    if out_temp.find('ok') < 0 and out_temp.find('error') < 0:
                                        print("DEBUG:", out_temp)
                                    else:
                                        grbl_out += out_temp;
                                        g_count += 1 # Iterate g-code counter
                                        grbl_out += str(g_count); # Add line finished indicator
                                        del c_line[0] # Delete the block character count corresponding to the last 'ok'

                                print("SND: " + str(l_count) + " : " + l_block,)
                                self.ser.write(str.encode(f'{l_block}\n')) # Send g-code block to grbl
                                #time.sleep(0.1)
                                print(f"BUF:{str(sum(c_line))} REC:{grbl_out}")
                                return_msg += f"{grbl_out}\n"


                            print("G-code streaming finished!\n")
                            print("WARNING: Wait until grbl completes buffered g-code blocks before exiting.")

                            return SendCommandResponse(return_msg, True)
                        else:
                            if self.rec_msg =="settings":
                                self.rec_msg = "$$"
                            print(f'SENDING:{self.rec_msg}')
                            bytes = self.ser.write(str.encode(f'{self.rec_msg}\r\n'))
                            print(f"BYTES SENT:{bytes}")
                            time.sleep(0.1)
                            bytesToRead = self.ser.inWaiting()
                            print("BYTES RECEIVED: ", bytesToRead)
                            data_raw = self.ser.read(bytesToRead).decode('utf-8')
                            str_data_raw = str(data_raw)
                            return SendCommandResponse(str_data_raw, True) #uncomment this line later
                            # split_data = str_data_raw.split('\\r\\n')#comment the rest and return str_data_raw
                            # print("RECEIVED [INFO: %s]: " % rospy.get_time())
                            # for i in split_data:
                            #     print(i)
                        #print("CLOSING PORT")
                        #self.ser.close()

                except Exception as e1:
                    print("error communicating....:" +str(e1))
        except Exception as e:
            print("COULDNT OPEN PORT, ERROR: "+ str(e))

def main():
    rospy.init_node('serial_com')
    SerialCom()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
