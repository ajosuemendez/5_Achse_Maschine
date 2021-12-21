#!/usr/bin/env python
import rospy
import serial
import time
import sys
from std_msgs.msg import String

class SerialCom():
    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.ser = self.init_ser()
        self.subscriber = rospy.Subscriber("/chatter", String, self.callback)
        rospy.on_shutdown(self.shutdown)

    def init_ser(self):
        return serial.serial_for_url(self.port, self.baudrate)

    def shutdown(self):
        print("\nCLOSING PORT: " + self.port)
        try:
            self.ser.close()
        except Exception as e:
            print("ERROR: "+ str(e))

    def callback(self, msg):
        #if not self.ser.isOpen():
        #    try:
        #        self.ser.open()
        #    except Exception as e:
        #        print("Error open serial port:" + str(e))
        try:
            if self.ser.isOpen():
                try:
                    print ("OPPENING PORT: " + self.port)
                    #self.ser.flushInput()
                    #self.ser.flushOutput()
                    self.rec_msg = msg.data
                    if self.rec_msg:
                        print(f'SENDING:{self.rec_msg}')
                        bytes = self.ser.write(str.encode(f'{self.rec_msg}\r\n'))
                        print(f"BYTES SENT:{bytes}")
                        time.sleep(0.1)
                        bytesToRead = self.ser.inWaiting()
                        print("BYTES RECEIVED: ", bytesToRead)
                        data_raw = self.ser.read(bytesToRead).decode('utf-8')
                        str_data_raw = str(data_raw)
                        split_data = str_data_raw.split('\\r\\n')
                        print("RECEIVED [INFO: %s]: " % rospy.get_time())
                        for i in split_data:
                            print(i)
                    #print("CLOSING PORT")
                    #self.ser.close()
                except Exception as e1:
                    print("error communicating....:" +str(e1))
        except Exception as e:
            print("COULDNT OPEN PORT, ERROR: "+ str(e))

def main():
    SerialCom()
    rospy.init_node('serial_com')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
