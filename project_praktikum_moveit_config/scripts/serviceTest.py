#!/usr/bin/env python

from __future__ import print_function

from project_praktikum_moveit_config.srv import SendCommand,SendCommandResponse
import rospy

class Server():
    def __init__(self):
        self.server = rospy.Service("/cmd_input", SendCommand, self.callback)

    def callback(self,req):
        arduino_msg = "ok"
        response = f"Message Received From Arduino: {arduino_msg}"
        return SendCommandResponse(response, True)

def main():
    try:
        server = Server()
        rospy.init_node("Server", anonymous=True)
        rospy.loginfo("Input Server NODE IS READY")
        rospy.spin()
    except rospy.ROSInterruptException:
        return


if __name__ == "__main__":
    main()
