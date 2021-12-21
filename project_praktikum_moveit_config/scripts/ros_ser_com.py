#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=None)
    time.sleep(1.1)
    #time.sleep(0.1)
    while ser.isOpen():
        print ("OPPENING PORT")
        rec_msg = msg.data
        if rec_msg:
            print(f'SENDING:{rec_msg}')
            bytes = ser.write(str.encode(f'{rec_msg}\n'))
            print(f"BYTES SENT:{bytes}")
            time.sleep(0.1)

            bytesToRead = ser.inWaiting()
            data_raw = ser.read(bytesToRead)
            str_data_raw = str(data_raw)
            split_data = str_data_raw.split('\\r\\n')
            print("RECEIVED: ")
            for i in split_data:
                print(i)
        break
    print("CLOSING PORT")
    ser.close()

def serial_connection():
    rospy.init_node('serial_connection', anonymous = True)

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        serial_connection()
    except rospy.ROSInterruptException:
        pass
