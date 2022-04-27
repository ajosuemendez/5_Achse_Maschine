#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped, Point
import tf
import math
from std_srvs.srv import SetBool
from dummy_robot_moveit_config.srv import CalculateJoints, CalculateJointsResponse
from dummy_robot_moveit_config.msg import ExecuteDesiredPoseAction, ExecuteDesiredPoseFeedback, ExecuteDesiredPoseResult
from dummy_robot_moveit_config.msg import ExecuteCartesianDesiredPoseAction, ExecuteCartesianDesiredPoseFeedback, ExecuteCartesianDesiredPoseResult
from std_msgs.msg import Bool
import actionlib
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from nav_msgs.msg import Path
import time
#from pilz_robot_programming import *

# import multiprocessing
from threading import Thread


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()

        eef_link = move_group.get_end_effector_link()

        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        # self.move_group.set_planner_id("LIN")
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pose_goal = geometry_msgs.msg.PoseStamped()
        self.getCurrentPoseService = rospy.Service("/get_pose", SetBool, self.callback_getCurrentPose)
        self.calc_pose_service = rospy.Service("/calc_pose", CalculateJoints, self.callback_calc_pose)
        self.shutdown_subscriber = rospy.Subscriber("/shutdown_gui", Bool, self.shutdown)
        self.plan_cartesian_service = rospy.Service("/plan_cartesian", CalculateJoints, self.plan_cartesian)
        self.action_server_execute = actionlib.SimpleActionServer("execute_action", ExecuteDesiredPoseAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server_execute.start()
        self.cartesian_action_server_execute = actionlib.SimpleActionServer("execute_cartesian_action", ExecuteCartesianDesiredPoseAction, execute_cb=self.action_plan_cartesian, auto_start = False)
        self.cartesian_action_server_execute.start()
        self.scale_m = 0.001
        self.roll_input = 0
        self.publish_tcp_coordinate = rospy.Publisher("/tcp_coordinate_point", PointStamped, queue_size=10)


    def get_tcp_coordinate(self):
        current_pose_header = self.move_group.get_current_pose(end_effector_link="tcp")
        x = round(current_pose_header.pose.position.x, 5)
        y = round(current_pose_header.pose.position.y, 5)
        z = round(current_pose_header.pose.position.z, 5)
        header = current_pose_header.header

        point_stamped = PointStamped()
        point_stamped.header = header
        point = Point()
        point.x = x
        point.y = y
        point.z = z

        point_stamped.point = point

        return point_stamped


    def display_path(self,id,stop):
        while True:
            point_stamped = self.get_tcp_coordinate()
            self.publish_tcp_coordinate.publish(point_stamped)
            time.sleep(0.08)
            if stop():
                break

    def execute_cb(self,goal):
        success = True
        try:
            self.pose_goal = geometry_msgs.msg.PoseStamped()
            #self.move_group.set_max_velocity_scaling_factor(goal.sim_vel/100)

            quaternion = tf.transformations.quaternion_from_euler(math.radians(goal.roll_input) ,math.radians(goal.pitch_input) ,math.radians(goal.yaw_input))
            self.pose_goal.pose.orientation.x = quaternion[0]
            self.pose_goal.pose.orientation.y = quaternion[1]
            self.pose_goal.pose.orientation.z = quaternion[2]
            self.pose_goal.pose.orientation.w = quaternion[3]
            self.pose_goal.pose.position.x = goal.x_input*self.scale_m
            self.pose_goal.pose.position.y = goal.y_input*self.scale_m
            self.pose_goal.pose.position.z = goal.z_input*self.scale_m

            self.move_group.set_pose_target(self.pose_goal.pose)
            chosen_constraint = goal.constraint

            self.move_group.set_path_constraints(None)

            if chosen_constraint == "None":
                ########DISABLING ALL CONSTRAINTS##############
                self.move_group.set_path_constraints(None)


            elif chosen_constraint == "Orientation":
                ####ORIENTATION CONSTRAINT########
                self.upright_constraints = Constraints()
                self.upright_constraints.name = "upright"
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header = self.pose_goal.header
                orientation_constraint.link_name = self.move_group.get_end_effector_link()
                orientation_constraint.orientation = self.pose_goal.pose.orientation
                orientation_constraint.absolute_x_axis_tolerance = goal.orientation_constraints_values[0] #0.4
                orientation_constraint.absolute_y_axis_tolerance = goal.orientation_constraints_values[1] #0.4
                orientation_constraint.absolute_z_axis_tolerance = goal.orientation_constraints_values[2] #0.4
                orientation_constraint.weight = 1
                #######ENABLING ORIENTATION CONSTGRAINT########
                self.upright_constraints.orientation_constraints.append(orientation_constraint)
                self.move_group.set_path_constraints(self.upright_constraints)

            elif chosen_constraint == "Position":
                ####POISTION CONSTRAINT###########
                self.fixed_point_constraint = Constraints()
                self.fixed_point_constraint.name = "fixed_point"
                point_constraint = PositionConstraint()
                point_constraint.header = self.pose_goal.header
                point_constraint.link_name = self.move_group.get_end_effector_link()
                point_constraint.target_point_offset = self.pose_goal.pose.position
                bounding_region = SolidPrimitive()
                bounding_region.type = SolidPrimitive.SPHERE
                bounding_region.dimensions.append(goal.position_constraint_value) #0.01
                point_constraint.constraint_region.primitives.append(bounding_region)
                point_constraint.constraint_region.primitive_poses.append(self.pose_goal.pose)
                point_constraint.weight = 1
                #######ENABLING POSITION CONSTGRAINT########
                self.fixed_point_constraint.position_constraints.append(point_constraint)
                self.move_group.set_path_constraints(self.fixed_point_constraint)

            elif chosen_constraint == "Joint":
                ######JOINT1 COINSTRAINT#############
                # self.fixed_base_constraint = Constraints()
                # self.fixed_base_constraint.name= "fixed_base"
                # joint_constraint = JointConstraint()
                # joint_constraint.joint_name = self.move_group.get_joints()[1]
                # joint_constraint.position = self.move_group.get_current_joint_values()[0]
                # #print(joint_constraint.position)
                # joint_constraint.tolerance_above = 0.174533
                # joint_constraint.tolerance_below = 0.174533
                # joint_constraint.weight = 1
                # #######ENABLING JOINT CONSTGRAINT########
                # self.fixed_base_constraint.joint_constraints.append(joint_constraint)
                # self.move_group.set_path_constraints(self.fixed_base_constraint)

                print("im in joint constraint")
                ######JOINT1 COINSTRAINT#############
                self.fixed_base_constraint = Constraints()
                self.fixed_base_constraint.name= "fixed_base"
                #joint X cosntraint
                joint_constraint_X = JointConstraint()
                joint_constraint_X.joint_name = self.move_group.get_joints()[1]
                joint_constraint_X.position = self.move_group.get_current_joint_values()[0]
                #print(joint_constraint_X.position)
                joint_constraint_X.tolerance_above = math.radians(goal.constraints_values[0]) #80 deg
                joint_constraint_X.tolerance_below = math.radians(goal.constraints_values[1]) #80 deg
                joint_constraint_X.weight = 1
                if goal.enable_joints[0] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_X)
                    print("joint X Enabled")
                #joint Y cosntraint
                joint_constraint_Y = JointConstraint()
                joint_constraint_Y.joint_name = self.move_group.get_joints()[2]
                joint_constraint_Y.position = self.move_group.get_current_joint_values()[1]
                #print(joint_constraint_Y.position)
                joint_constraint_Y.tolerance_above = math.radians(goal.constraints_values[2]) #70 deg
                joint_constraint_Y.tolerance_below = math.radians(goal.constraints_values[3]) #70 deg
                joint_constraint_Y.weight = 1
                if goal.enable_joints[1] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_Y)
                    print("joint Y Enabled")
                #joint Z cosntraint
                joint_constraint_Z = JointConstraint()
                joint_constraint_Z.joint_name = self.move_group.get_joints()[3]
                joint_constraint_Z.position = self.move_group.get_current_joint_values()[2]
                #print(joint_constraint_Z.position)
                joint_constraint_Z.tolerance_above = math.radians(goal.constraints_values[4]) #40 deg
                joint_constraint_Z.tolerance_below = math.radians(goal.constraints_values[5]) #10 deg
                joint_constraint_Z.weight = 1
                if goal.enable_joints[2] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_Z)
                    print("joint Z Enabled")
                #joint A cosntraint
                joint_constraint_A = JointConstraint()
                joint_constraint_A.joint_name = self.move_group.get_joints()[4]
                joint_constraint_A.position = self.move_group.get_current_joint_values()[3]
                #print(joint_constraint_A.position)
                joint_constraint_A.tolerance_above = math.radians(goal.constraints_values[6]) #45 deg
                joint_constraint_A.tolerance_below = math.radians(goal.constraints_values[7]) #45 deg
                joint_constraint_A.weight = 1
                if goal.enable_joints[3] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_A)
                    print("joint A Enabled")
                #joint B cosntraint
                joint_constraint_B = JointConstraint()
                joint_constraint_B.joint_name = self.move_group.get_joints()[5]
                joint_constraint_B.position = self.move_group.get_current_joint_values()[4]
                #print(joint_constraint_B.position)
                joint_constraint_B.tolerance_above = math.radians(goal.constraints_values[8]) #45 deg
                joint_constraint_B.tolerance_below = math.radians(goal.constraints_values[9]) #45 deg
                joint_constraint_B.weight = 1
                if goal.enable_joints[4] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_B)
                    print("joint B Enabled")
                #joint C cosntraint
                joint_constraint_C = JointConstraint()
                joint_constraint_C.joint_name = self.move_group.get_joints()[6]
                joint_constraint_C.position = self.move_group.get_current_joint_values()[5]
                #print(joint_constraint_C.position)
                joint_constraint_C.tolerance_above = math.radians(goal.constraints_values[10]) #45 deg
                joint_constraint_C.tolerance_below = math.radians(goal.constraints_values[11]) #45 deg
                joint_constraint_C.weight = 1

                if goal.enable_joints[5] == 1:
                    self.fixed_base_constraint.joint_constraints.append(joint_constraint_C)
                    print("joint C Enabled")

                #######ENABLING JOINT CONSTGRAINT########
                self.move_group.set_path_constraints(self.fixed_base_constraint)

            calc_plan = self.move_group.plan()

        except Exception as e:
            print(e)
            success = False


        feedback_action = ExecuteDesiredPoseFeedback()
        result_action = ExecuteDesiredPoseResult()
        rate = rospy.Rate(100)

        self.action_server_execute.set_succeeded(result_action)

        if goal.command: ##command true means normal execution
            feedback_action.feedback = "Executing....."
            self.move(sim_vel=goal.sim_vel)
            self.action_server_execute.publish_feedback(feedback_action)
            feedback_action.feedback = "Finished Executing....."
            self.action_server_execute.publish_feedback(feedback_action)

            if success:
                modified_coodinates = []
                for i in range(len(calc_plan[1].joint_trajectory.points)):
                    for j in range(6):
                        modified_coodinates.append(round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[j]),2))

                result_action.result = modified_coodinates
                self.action_server_execute.set_succeeded(result_action)
            else:
                modified_coodinates = []
                result_action.result = modified_coodinates
                self.action_server_execute.set_succeeded(result_action)




    def action_plan_cartesian(self,goal):
        #self.move_group.set_max_velocity_scaling_factor(goal.sim_vel/100)
        success = True
        feedback_action = ExecuteCartesianDesiredPoseFeedback()
        result_action = ExecuteCartesianDesiredPoseResult()

        if goal.command: ##command true means normal execution
            waypoints = []

            wpose = self.move_group.get_current_pose(end_effector_link="tcp").pose

            wpose.position.x = goal.x_input*self.scale_m
            wpose.position.y = goal.y_input*self.scale_m
            wpose.position.z = goal.z_input*self.scale_m

            waypoints.append(copy.deepcopy(wpose))

            fraction = 0.0
            attempts = 0
            max_tries = 100
            success = False

            while fraction < 1.0 and attempts < max_tries:
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.004, 0.0)
                attempts += 1
                if fraction == 1.0:
                     success = True

            print("fraction",fraction)

            if success:
                feedback_action.feedback = "Executing....."
                self.cartesian_action_server_execute.publish_feedback(feedback_action)
                self.move(sim_vel=goal.sim_vel, normal_execution=False, plan=plan)
                feedback_action.feedback = "Finished Executing....."
                self.cartesian_action_server_execute.publish_feedback(feedback_action)

                modified_coodinates = []
                for i in range(len(plan.joint_trajectory.points)):
                    for j in range(6):
                        modified_coodinates.append(round(math.degrees(plan.joint_trajectory.points[i].positions[j]),2))
                result_action.result = modified_coodinates
                self.cartesian_action_server_execute.set_succeeded(result_action)

            else:
                modified_coodinates = []
                result_action.result = modified_coodinates
                self.cartesian_action_server_execute.set_succeeded(result_action)



    def plan_cartesian(self, req):
        waypoints = []

        wpose = self.move_group.get_current_pose(end_effector_link="tcp").pose

        wpose.position.x = req.x_input*self.scale_m
        wpose.position.y = req.y_input*self.scale_m
        wpose.position.z = req.z_input*self.scale_m

        waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0
        attempts = 0
        max_tries = 100
        success = False
        modified_coodinates = [1,2,3,4,5,6]
        print("trying to get coordinate")
        print("there is some problem")

        while fraction < 1.0 and attempts < max_tries:
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.004, 0.0)
            attempts += 1
            if fraction == 1.0:
                 success = True

        if success:
            if req.check == 0:
                self.move_group.execute(plan, wait=True)
            modified_coodinates = []
            for i in range(len(plan.joint_trajectory.points)):
                for j in range(6):
                    modified_coodinates.append(plan.joint_trajectory.points[i].positions[j])

        return CalculateJointsResponse(modified_coodinates, success)

    def shutdown(self,msg):
        if msg.data:
            rospy.signal_shutdown("GUI was shutdown")
            sys.exit(1)

    def callback_calc_pose(self,req):
        try:
            current_joints_list = []
            calculated_joints_list = []
            valid_plan = False
            self.pose_goal = geometry_msgs.msg.PoseStamped()
            self.move_group.set_max_velocity_scaling_factor(0.5)

            quaternion = tf.transformations.quaternion_from_euler(math.radians(req.roll_input) ,math.radians(req.pitch_input) ,math.radians(req.yaw_input)) ##added-1 and -1
            self.pose_goal.pose.orientation.x = quaternion[0]
            self.pose_goal.pose.orientation.y = quaternion[1]
            self.pose_goal.pose.orientation.z = quaternion[2]
            self.pose_goal.pose.orientation.w = quaternion[3]
            self.pose_goal.pose.position.x = req.x_input*self.scale_m
            self.pose_goal.pose.position.y = req.y_input*self.scale_m
            self.pose_goal.pose.position.z = req.z_input*self.scale_m

            self.move_group.set_pose_target(self.pose_goal.pose)
            self.move_group.set_path_constraints(None)


            if req.check!=1:
                if req.constraint == "Joint":
                    print("im in joint constraint")
                    ######JOINT1 COINSTRAINT#############
                    self.fixed_base_constraint = Constraints()
                    self.fixed_base_constraint.name= "fixed_base"
                    #joint X cosntraint
                    joint_constraint_X = JointConstraint()
                    joint_constraint_X.joint_name = self.move_group.get_joints()[1]
                    joint_constraint_X.position = self.move_group.get_current_joint_values()[0]
                    #print(joint_constraint_X.position)
                    joint_constraint_X.tolerance_above = math.radians(req.constraints_values[0]) #80 deg
                    joint_constraint_X.tolerance_below = math.radians(req.constraints_values[1]) #80 deg
                    joint_constraint_X.weight = 1
                    if req.enable_joints[0] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_X)
                        print("joint X Enabled")
                    #joint Y cosntraint
                    joint_constraint_Y = JointConstraint()
                    joint_constraint_Y.joint_name = self.move_group.get_joints()[2]
                    joint_constraint_Y.position = self.move_group.get_current_joint_values()[1]
                    #print(joint_constraint_Y.position)
                    joint_constraint_Y.tolerance_above = math.radians(req.constraints_values[2]) #70 deg
                    joint_constraint_Y.tolerance_below = math.radians(req.constraints_values[3]) #70 deg
                    joint_constraint_Y.weight = 1
                    if req.enable_joints[1] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_Y)
                        print("joint Y Enabled")
                    #joint Z cosntraint
                    joint_constraint_Z = JointConstraint()
                    joint_constraint_Z.joint_name = self.move_group.get_joints()[3]
                    joint_constraint_Z.position = self.move_group.get_current_joint_values()[2]
                    #print(joint_constraint_Z.position)
                    joint_constraint_Z.tolerance_above = math.radians(req.constraints_values[4]) #40 deg
                    joint_constraint_Z.tolerance_below = math.radians(req.constraints_values[5]) #10 deg
                    joint_constraint_Z.weight = 1
                    if req.enable_joints[2] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_Z)
                        print("joint Z Enabled")
                    #joint A cosntraint
                    joint_constraint_A = JointConstraint()
                    joint_constraint_A.joint_name = self.move_group.get_joints()[4]
                    joint_constraint_A.position = self.move_group.get_current_joint_values()[3]
                    #print(joint_constraint_A.position)
                    joint_constraint_A.tolerance_above = math.radians(req.constraints_values[6]) #45 deg
                    joint_constraint_A.tolerance_below = math.radians(req.constraints_values[7]) #45 deg
                    joint_constraint_A.weight = 1
                    if req.enable_joints[3] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_A)
                        print("joint A Enabled")
                    #joint B cosntraint
                    joint_constraint_B = JointConstraint()
                    joint_constraint_B.joint_name = self.move_group.get_joints()[5]
                    joint_constraint_B.position = self.move_group.get_current_joint_values()[4]
                    #print(joint_constraint_B.position)
                    joint_constraint_B.tolerance_above = math.radians(req.constraints_values[8]) #45 deg
                    joint_constraint_B.tolerance_below = math.radians(req.constraints_values[9]) #45 deg
                    joint_constraint_B.weight = 1
                    if req.enable_joints[4] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_B)
                        print("joint B Enabled")
                    #joint C cosntraint
                    joint_constraint_C = JointConstraint()
                    joint_constraint_C.joint_name = self.move_group.get_joints()[6]
                    joint_constraint_C.position = self.move_group.get_current_joint_values()[5]
                    #print(joint_constraint_C.position)
                    joint_constraint_C.tolerance_above = math.radians(req.constraints_values[10]) #45 deg
                    joint_constraint_C.tolerance_below = math.radians(req.constraints_values[11]) #45 deg
                    joint_constraint_C.weight = 1

                    if req.enable_joints[5] == 1:
                        self.fixed_base_constraint.joint_constraints.append(joint_constraint_C)
                        print("joint C Enabled")

                    #######ENABLING JOINT CONSTGRAINT########

                    self.move_group.set_path_constraints(self.fixed_base_constraint)

                elif req.constraint == "Orientation":
                    #####ORIENTATION CONSTRAINT########
                    self.upright_constraints = Constraints()
                    self.upright_constraints.name = "upright"
                    orientation_constraint = OrientationConstraint()
                    orientation_constraint.header = self.pose_goal.header
                    orientation_constraint.link_name = self.move_group.get_end_effector_link()
                    orientation_constraint.orientation = self.pose_goal.pose.orientation
                    orientation_constraint.absolute_x_axis_tolerance = req.orientation_constraints_values[0] #0.4
                    orientation_constraint.absolute_y_axis_tolerance = req.orientation_constraints_values[1] #0.4
                    orientation_constraint.absolute_z_axis_tolerance = req.orientation_constraints_values[2] #0.4
                    orientation_constraint.weight = 1
                    #######ENABLING ORIENTATION CONSTGRAINT########
                    self.upright_constraints.orientation_constraints.append(orientation_constraint)
                    self.move_group.set_path_constraints(self.upright_constraints)

                elif req.constraint == "Position":
                    ####POISTION CONSTRAINT###########
                    self.fixed_point_constraint = Constraints()
                    self.fixed_point_constraint.name = "fixed_point"
                    point_constraint = PositionConstraint()
                    point_constraint.header = self.pose_goal.header
                    point_constraint.link_name = self.move_group.get_end_effector_link()
                    point_constraint.target_point_offset = self.pose_goal.pose.position
                    bounding_region = SolidPrimitive()
                    bounding_region.type = SolidPrimitive.SPHERE
                    bounding_region.dimensions.append(req.position_constraint_value) #0.01
                    point_constraint.constraint_region.primitives.append(bounding_region)
                    point_constraint.constraint_region.primitive_poses.append(self.pose_goal.pose)
                    point_constraint.weight = 1
                    #######ENABLING POSITION CONSTGRAINT########
                    self.fixed_point_constraint.position_constraints.append(point_constraint)
                    self.move_group.set_path_constraints(self.fixed_point_constraint)

                if req.constraint == "None":
                    ########DISABLING ALL CONSTRAINTS##############
                    self.move_group.set_path_constraints(None)



            else:
                self.move_group.set_path_constraints(None)


            calc_plan = self.move_group.plan()
            length = len(calc_plan[1].joint_trajectory.points)

            if length:
                valid_plan = True

                for i in range(0,-2,-1):
                    joint_1 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[0]), 2)
                    joint_2 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[1]),2)
                    joint_3 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[2]),2)
                    joint_4 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[3]),2)
                    joint_5 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[4]),2)
                    joint_6 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[5]),2)

                    array = [joint_1, joint_2, joint_3, joint_4,joint_5, joint_6]

                    for k in array:
                        if i== 0:
                            current_joints_list.append(k)
                        else:
                            calculated_joints_list.append(k)

            if req.check!=1 and req.simulate:
                self.move()

            return CalculateJointsResponse([*current_joints_list, *calculated_joints_list], valid_plan)

        except Exception as e:
            print(e)
            return CalculateJointsResponse([0,0,0,0,0,0,0,0,0,0], False)


    def move(self, sim_vel=90, normal_execution=True, plan=None):
        self.move_group.set_max_velocity_scaling_factor(sim_vel/100)
        stop_threads = False
        p2 = Thread(target=self.display_path, args=(1, lambda: stop_threads))
        p2.start()
        print("about to start movoving object")
        if normal_execution:
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
        else:
            self.move_group.execute(plan, wait=True)
        stop_threads = True
        p2.join()


    def callback_getCurrentPose(self,req):
        if req.data:
            current_pose = self.move_group.get_current_pose().pose
            x = round(current_pose.position.x * 1000,2)
            y = round(current_pose.position.y * 1000,2)
            z = round(current_pose.position.z * 1000,2)
            quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            roll = round(math.degrees(roll),2)
            pitch = round(math.degrees(pitch),2)
            yaw = round(math.degrees(yaw),2)

            return True, f"X{x}Y{y}Z{z}Roll{roll}Pitch{pitch}Yaw{yaw}"
        return False, "Execution Failed"


def main():
    try:
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        tutorial = MoveGroupPythonInterfaceTutorial()
        rospy.loginfo("EXECUTE_POSE NODE IS READY")
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
