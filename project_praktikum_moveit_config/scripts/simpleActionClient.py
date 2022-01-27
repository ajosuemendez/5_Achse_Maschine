#! /usr/bin/env python

import rospy
import actionlib
from project_praktikum_moveit_config.msg import SerialCommunicationAction, SerialCommunicationGoal

class Client():
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('serial_com', SerialCommunicationAction)

    def send_action(self):
        self.action_client.wait_for_server()
        goal = SerialCommunicationGoal(command="5")
        self.action_client.send_goal(goal, feedback_cb=self.feedback_cb)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    def feedback_cb(self, msg):
        print("Feedback received: ", msg)


if __name__ == '__main__':
    try:
        rospy.init_node('action_client_node')
        client = Client()
        result = client.send_action()
        print("Final Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
