#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


def all_close(goal, actual, tolerance):
    all_equal= True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def plan_cartesian_path(self, x_input, y_input, z_input, r_input, p_input, yaw_input):

        waypoints = []

        scale=1
        scale_m = 0.001

        wpose = self.group.get_current_pose(end_effector_link="eff").pose
        wpose = self.group.get_current_pose(end_effector_link="eff").pose
        waypoints.append(copy.deepcopy(wpose))
        #wpose.position.z += scale * 0.1  # First move up (z)
        #wpose.position.y -= scale * 0.05 # and sideways (y)
        wpose.position.x = scale * (float(x_input)*scale_m)
        wpose.position.y = scale * (float(y_input)*scale_m)
        wpose.position.z = scale * (float(z_input)*scale_m)

        #orientation_list = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]

        #(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #roll = scale * math.radians(float(r_input))
        #pitch = scale * math.radians(float(p_input))
        #yaw = scale * math.radians(float(yaw_input))*(-1)

        #quaternion = quaternion_from_euler(float(roll) ,float(pitch) ,float(yaw))
        #print(quaternion)
        wpose.orientation.x = 0.01# quaternion[0]
        wpose.orientation.y = 0.01#quaternion[1]
        wpose.orientation.z = 0.01#quaternion[2]
        wpose.orientation.w = 0.99#quaternion[3]

        waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0
        attempts = 0
        max_tries = 100
        success = False

        while fraction < 1.0 and attempts < max_tries:
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
            attempts += 1
            if fraction == 1.0:
                 success = True

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction, success, attempts


    def execute_plan(self, plan):
        self.group.execute(plan, wait=True)

def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
        x = 500#input("enter x in mm: ") #mm
        y = 550#input("enter y in mm: ")#mm
        z = 500#input("enter z in mm: ") #mm
        r = 0 #degrees
        p = 0 #(degrees)
        yaw = 0 #degrees

        cartesian_plan, fraction, success, attempts = tutorial.plan_cartesian_path(x_input=x, y_input=y, z_input=z, r_input=r, p_input=p, yaw_input=yaw)

        if success:
            print(f"Completed Trajectory Planned And To Be Executed After {attempts} Attempts")
            tutorial.execute_plan(cartesian_plan)
        else:
            print(f"No proper Cartesian Plan Found after {attempts} Attempts!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
