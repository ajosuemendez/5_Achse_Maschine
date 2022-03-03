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
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pose_goal = geometry_msgs.msg.PoseStamped()
        self.getCurrentPoseService = rospy.Service("/get_pose", SetBool, self.callback_getCurrentPose)
        self.calc_pose_service = rospy.Service("/calc_pose", CalculateJoints, self.callback_calc_pose)
        self.execute_pose_service = rospy.Service("/execute_pose", SetBool, self.execute_pose_goal)
        self.shutdown_subscriber = rospy.Subscriber("/shutdown_gui", Bool, self.shutdown)
        self.plan_cartesian_service = rospy.Service("/plan_cartesian", CalculateJoints, self.plan_cartesian)
        self.action_server_execute = actionlib.SimpleActionServer("execute_action", ExecuteDesiredPoseAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server_execute.start()
        self.cartesian_action_server_execute = actionlib.SimpleActionServer("execute_cartesian_action", ExecuteCartesianDesiredPoseAction, execute_cb=self.action_plan_cartesian, auto_start = False)
        self.cartesian_action_server_execute.start()
        self.scale_m = 0.001
        self.roll_input = 0


    def execute_cb(self,goal):
        success = True
        try:
            # current_joints_list = []
            # calculated_joints_list = []
            # valid_plan = False
            self.pose_goal = geometry_msgs.msg.PoseStamped()
            #print("my joints values:",self.move_group.get_current_joint_values())
            self.move_group.set_max_velocity_scaling_factor(goal.sim_vel/100)

            quaternion = tf.transformations.quaternion_from_euler(math.radians(goal.roll_input) ,math.radians(goal.pitch_input) ,math.radians(goal.yaw_input)) ##added-1 and -1
            self.pose_goal.pose.orientation.x = quaternion[0]
            self.pose_goal.pose.orientation.y = quaternion[1]
            self.pose_goal.pose.orientation.z = quaternion[2]
            self.pose_goal.pose.orientation.w = quaternion[3]
            self.pose_goal.pose.position.x = goal.x_input*self.scale_m
            self.pose_goal.pose.position.y = goal.y_input*self.scale_m
            self.pose_goal.pose.position.z = goal.z_input*self.scale_m

            self.move_group.set_pose_target(self.pose_goal.pose)
            ####ORIENTATION CONSTRAINT########
            self.upright_constraints = Constraints()
            self.upright_constraints.name = "upright"
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = self.pose_goal.header
            orientation_constraint.link_name = self.move_group.get_end_effector_link()
            orientation_constraint.orientation = self.pose_goal.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.4
            orientation_constraint.absolute_y_axis_tolerance = 0.4
            orientation_constraint.absolute_z_axis_tolerance = 0.4
            orientation_constraint.weight = 1

            #######ENABLING ORIENTATION CONSTGRAINT########
            #self.upright_constraints.orientation_constraints.append(orientation_constraint)
            #self.move_group.set_path_constraints(self.upright_constraints)

            ######JOINT1 COINSTRAINT#############
            self.fixed_base_constraint = Constraints()
            self.fixed_base_constraint.name= "fixed_base"
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = self.move_group.get_joints()[1]
            joint_constraint.position = self.move_group.get_current_joint_values()[0]
            print(joint_constraint.position)
            joint_constraint.tolerance_above = 0.174533
            joint_constraint.tolerance_below = 0.174533
            joint_constraint.weight = 1
            #######ENABLING JOINT CONSTGRAINT########
            #self.fixed_base_constraint.joint_constraints.append(joint_constraint)
            #self.move_group.set_path_constraints(self.fixed_base_constraint)

            ####POISTION CONSTRAINT###########
            self.fixed_point_constraint = Constraints()
            self.fixed_point_constraint.name = "fixed_point"
            point_constraint = PositionConstraint()
            point_constraint.header = self.pose_goal.header
            point_constraint.link_name = self.move_group.get_end_effector_link()
            point_constraint.target_point_offset = self.pose_goal.pose.position
            bounding_region = SolidPrimitive()
            bounding_region.type = 2
            bounding_region.dimensions.append(0.001)
            point_constraint.constraint_region.primitives.append(bounding_region)
            point_constraint.constraint_region.primitive_poses.append(self.pose_goal.pose)
            point_constraint.weight = 1
            #######ENABLING POSITION CONSTGRAINT########
            self.fixed_point_constraint.position_constraints.append(point_constraint)
            #self.fixed_point_constraint.joint_constraints.append(joint_constraint)
            self.move_group.set_path_constraints(self.fixed_point_constraint)

            ########DISABLING ALL CONSTRAINTS##############
            #self.move_group.set_path_constraints(None)

            # my_scale = 1000
            # calc_plan_1 = self.move_group.plan()
            # print("plan1:", calc_plan_1[1].joint_trajectory.points[-1].positions)
            # calc_plan_2 = self.move_group.plan()
            # print("plan2:", calc_plan_2[1].joint_trajectory.points[-1].positions)
            # joint1 = calc_plan_1[1].joint_trajectory.points[-1].positions[0]
            # joint2 = calc_plan_1[1].joint_trajectory.points[-1].positions[1]
            # joint3 = calc_plan_1[1].joint_trajectory.points[-1].positions[2]
            # joint4 = calc_plan_1[1].joint_trajectory.points[-1].positions[3]
            # joint5 = calc_plan_1[1].joint_trajectory.points[-1].positions[4]
            # joint6 = calc_plan_1[1].joint_trajectory.points[-1].positions[5]
            #
            # next_joint1 = calc_plan_2[1].joint_trajectory.points[-1].positions[0]
            # next_joint2 = calc_plan_2[1].joint_trajectory.points[-1].positions[1]
            # next_joint3 = calc_plan_2[1].joint_trajectory.points[-1].positions[2]
            # next_joint4 = calc_plan_2[1].joint_trajectory.points[-1].positions[3]
            # next_joint5 = calc_plan_2[1].joint_trajectory.points[-1].positions[4]
            # next_joint6 = calc_plan_2[1].joint_trajectory.points[-1].positions[5]
            #
            # abs_joint1 = abs(joint1 - next_joint1)
            # abs_joint2 = abs(joint2 - next_joint2)
            # abs_joint3 = abs(joint3 - next_joint3)
            # abs_joint4 = abs(joint4 - next_joint4)
            # abs_joint5 = abs(joint5 - next_joint5)
            # abs_joint6 = abs(joint6 - next_joint6)
            #
            # thresshold = 0.17
            # counting = 0
            #
            # while (abs_joint1>thresshold) or (abs_joint2>thresshold) or (abs_joint3>thresshold) or (abs_joint4>thresshold) or (abs_joint5>thresshold) or (abs_joint6>thresshold):
            #     calc_plan_3 = self.move_group.plan()
            #
            #     next_joint1 = calc_plan_3[1].joint_trajectory.points[-1].positions[0]
            #     next_joint2 = calc_plan_3[1].joint_trajectory.points[-1].positions[1]
            #     next_joint3 = calc_plan_3[1].joint_trajectory.points[-1].positions[2]
            #     next_joint4 = calc_plan_3[1].joint_trajectory.points[-1].positions[3]
            #     next_joint5 = calc_plan_3[1].joint_trajectory.points[-1].positions[4]
            #     next_joint6 = calc_plan_3[1].joint_trajectory.points[-1].positions[5]
            #
            #     abs_joint1 = abs(joint1 - next_joint1)
            #     abs_joint2 = abs(joint2 - next_joint2)
            #     abs_joint3 = abs(joint3 - next_joint3)
            #     abs_joint5 = abs(joint5 - next_joint5)
            #     abs_joint4 = abs(joint4 - next_joint4)
            #     abs_joint6 = abs(joint6 - next_joint6)
            #
            #     counting +=1
            #
            # print("plan3:", calc_plan_3[1].joint_trajectory.points[-1].positions)
            # print("abs_joint1:",abs_joint1)
            # print("abs_joint2:",abs_joint2)
            # print("abs_joint3:",abs_joint3)
            # print("abs_joint4:",abs_joint4)
            # print("abs_joint5:",abs_joint5)
            # print("abs_joint6:",abs_joint6)
            #
            # print("counter:", counting)
            calc_plan = self.move_group.plan()

        except Exception as e:
            print(e)
            success = False


        feedback_action = ExecuteDesiredPoseFeedback()
        result_action = ExecuteDesiredPoseResult()
        rate = rospy.Rate(100)

        if goal.command: ##command true means normal execution
            feedback_action.feedback = "Executing....."
            plan = self.move_group.go(wait=True)
            #self.move_group.execute(calc_plan, wait=True)
            #print("plan:",plan)
            self.action_server_execute.publish_feedback(feedback_action)
            self.move_group.stop()
            feedback_action.feedback = "Finished Executing....."
            self.action_server_execute.publish_feedback(feedback_action)
            self.move_group.clear_pose_targets()
            current_pose = self.move_group.get_current_pose().pose
            all_close(self.pose_goal.pose, current_pose, 0.01)

            if success:
                # result_action.result = "Execution Successfuly"
                # self.action_server_execute.set_succeeded(result_action)
                modified_coodinates = []
                #print("here starts!!")
                #print(calc_plan[1].joint_trajectory.points)
                for i in range(len(calc_plan[1].joint_trajectory.points)):
                    print("Positions: ",calc_plan[1].joint_trajectory.points[i].positions)
                    for j in range(6):
                        modified_coodinates.append(round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[j]),2))

                result_action.result = modified_coodinates
                print("result: ",result_action.result)
                self.action_server_execute.set_succeeded(result_action)
            else:
                modified_coodinates = []
                result_action.result = modified_coodinates
                self.action_server_execute.set_succeeded(result_action)




    def action_plan_cartesian(self,goal):
        self.move_group.set_max_velocity_scaling_factor(goal.sim_vel/100)
        success = True
        feedback_action = ExecuteCartesianDesiredPoseFeedback()
        result_action = ExecuteCartesianDesiredPoseResult()

        if goal.command: ##command true means normal execution
            waypoints = []

            wpose = self.move_group.get_current_pose(end_effector_link="tcp").pose

            wpose.position.x = goal.x_input*self.scale_m
            wpose.position.y = goal.y_input*self.scale_m
            wpose.position.z = goal.z_input*self.scale_m

            # orientation_list = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
            #
            # # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
            # # roll += scale * math.radians(req.roll_input)
            # # pitch += scale * math.radians(req.pitch_input)
            # # yaw += scale * math.radians(req.yaw_input)*(-1)
            #
            # quaternion = tf.transformations.quaternion_from_euler(float(goal.roll_input) ,float(goal.pitch_input) ,float(goal.yaw_input))
            # wpose.orientation.x = quaternion[0]
            # wpose.orientation.y = quaternion[1]
            # wpose.orientation.z = quaternion[2]
            # wpose.orientation.w = quaternion[3]

            waypoints.append(copy.deepcopy(wpose))

            fraction = 0.0
            attempts = 0
            max_tries = 100
            success = False
            #print("trying to get coordinate")
            #print("is this is?")
            while fraction < 1.0 and attempts < max_tries:
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.004, 0.0)
                attempts += 1
                if fraction == 1.0:
                     success = True

            print("fraction",fraction)

            if success:
                feedback_action.feedback = "Executing....."
                self.cartesian_action_server_execute.publish_feedback(feedback_action)
                self.move_group.execute(plan, wait=True)
                feedback_action.feedback = "Finished Executing....."
                self.cartesian_action_server_execute.publish_feedback(feedback_action)
                # result_action.result = "Execution Successfuly"
                # self.cartesian_action_server_execute.set_succeeded(result_action)
                modified_coodinates = []
                for i in range(len(plan.joint_trajectory.points)):
                    for j in range(6):
                        modified_coodinates.append(round(math.degrees(plan.joint_trajectory.points[i].positions[j]),2))
                result_action.result = modified_coodinates
                self.cartesian_action_server_execute.set_succeeded(result_action)

            else:
                # result_action.result = "Execution Successfuly"
                # self.cartesian_action_server_execute.set_succeeded(result_action)
                modified_coodinates = []
                result_action.result = modified_coodinates
                self.cartesian_action_server_execute.set_succeeded(result_action)



    def plan_cartesian(self, req):
        waypoints = []

        wpose = self.move_group.get_current_pose(end_effector_link="tcp").pose

        wpose.position.x = req.x_input*self.scale_m
        wpose.position.y = req.y_input*self.scale_m
        wpose.position.z = req.z_input*self.scale_m

        # orientation_list = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
        #
        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # roll += scale * math.radians(req.roll_input)
        # pitch += scale * math.radians(req.pitch_input)
        # yaw += scale * math.radians(req.yaw_input)*(-1)
        #
        # quaternion = tf.transformations.quaternion_from_euler(float(roll) ,float(pitch) ,float(yaw))
        # wpose.orientation.x = quaternion[0]
        # wpose.orientation.y = quaternion[1]
        # wpose.orientation.z = quaternion[2]
        # wpose.orientation.w = quaternion[3]

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

        #print(success)
        #print(f"Number {i}: Pos:",plan.joint_trajectory.points[i].positions)

        # print(plan.joint_trajectory.points)

        if success:
            if req.check == 0:
                self.move_group.execute(plan, wait=True)
            modified_coodinates = []
            for i in range(len(plan.joint_trajectory.points)):
                for j in range(6):
                    modified_coodinates.append(plan.joint_trajectory.points[i].positions[j])
            #my_scale=1000

            #print(plan.joint_trajectory.points[-1])
            # modified_coodinates.append(round(my_scale*plan.joint_trajectory.points[-1].positions[1], 2))
            # modified_coodinates.append(round(my_scale*plan.joint_trajectory.points[-1].positions[0],2))
            # modified_coodinates.append(round(-my_scale*plan.joint_trajectory.points[-1].positions[2],2))
            # modified_coodinates.append(round((-1)*math.degrees(plan.joint_trajectory.points[-1].positions[4]),2))
            # modified_coodinates.append(0)


        #return CalculateJointsResponse(plan.joint_trajectory.points[-1].positions, success)
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

            quaternion = tf.transformations.quaternion_from_euler(math.radians(req.roll_input) ,math.radians(req.pitch_input) ,math.radians(req.yaw_input)) ##added-1 and -1
            self.pose_goal.pose.orientation.x = quaternion[0]
            self.pose_goal.pose.orientation.y = quaternion[1]
            self.pose_goal.pose.orientation.z = quaternion[2]
            self.pose_goal.pose.orientation.w = quaternion[3]
            self.pose_goal.pose.position.x = req.x_input*self.scale_m
            self.pose_goal.pose.position.y = req.y_input*self.scale_m
            self.pose_goal.pose.position.z = req.z_input*self.scale_m

            self.move_group.set_pose_target(self.pose_goal.pose)

            # my_scale = 1000
            calc_plan = self.move_group.plan()
            #print("CALC_PLAN:",calc_plan)
            length = len(calc_plan[1].joint_trajectory.points)
            #drehung = req.yaw_input//360.19

            if length:
                valid_plan = True
                #print(calc_plan[1].joint_trajectory.points)

                for i in range(0,-2,-1):
                    joint_1 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[0]), 2)
                    joint_2 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[1]),2)
                    joint_3 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[2]),2)
                    joint_4 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[3]),2)
                    joint_5 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[4]),2)
                    joint_6 = round(math.degrees(calc_plan[1].joint_trajectory.points[i].positions[5]),2)
                    # if math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3])<0.19 and math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3])>=0:
                    #     my_c = 0 + drehung*360
                    # else:
                    #     my_c = round(360-math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3]),2) + drehung*360

                    # my_b = round(-math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[4]),2)


                    array = [joint_1, joint_2, joint_3, joint_4,joint_5, joint_6]

                    for k in array:
                        if i== 0:
                            current_joints_list.append(k)
                        else:
                            calculated_joints_list.append(k)

            return CalculateJointsResponse([*current_joints_list, *calculated_joints_list], valid_plan)

        except Exception as e:
            print(e)
            return CalculateJointsResponse([0,0,0,0,0,0,0,0,0,0], False)





    def execute_pose_goal(self,req):
        if req.data:
            plan = self.move_group.go(wait=True)

            self.move_group.stop()

            self.move_group.clear_pose_targets()

            current_pose = self.move_group.get_current_pose().pose
            all_close(self.pose_goal.pose, current_pose, 0.01)

            return True, "Execution Successfuly"
        return False, "Execution Failed"

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
        MoveGroupPythonInterfaceTutorial()
        rospy.loginfo("EXECUTE_POSE NODE IS READY")
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
