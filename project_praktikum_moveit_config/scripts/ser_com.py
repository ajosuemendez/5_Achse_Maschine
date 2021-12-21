#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String
def serial_connection():
    rospy.init_node('serial_connection', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        while ser.isOpen():
            print ("Port Open")
            for i in range(1):
                ser.flushInput()
                ser.flushOutput()

                msg = input("Enter a command: ")
                if msg:
                    print(f'SENDING:{msg}')
                    bytes = ser.write(str.encode(f'{msg}\n'))
                    print(f"BYTES SENT:{bytes}")
                    time.sleep(1)
                    rate.sleep()
                    bytesToRead = ser.inWaiting()
                    data_raw = ser.read(bytesToRead)
                    str_data_raw = str(data_raw)
                    split_data = str_data_raw.split('\\r\\n')
                    print("RECEIVED: ")
                    for i in split_data:
                        print(i)
            break
        ser.close()

if __name__ == '__main__':
    try:
        serial_connection()
    except rospy.ROSInterruptException:
        pass
