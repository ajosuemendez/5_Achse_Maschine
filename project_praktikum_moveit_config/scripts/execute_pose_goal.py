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
from project_praktikum_moveit_config.srv import CalculateJoints, CalculateJointsResponse
from project_praktikum_moveit_config.msg import ExecuteDesiredPoseAction, ExecuteDesiredPoseFeedback, ExecuteDesiredPoseResult
from std_msgs.msg import Bool
import actionlib

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
        self.calc_pose_service = rospy.Service("/calc_pose", CalculateJoints, self.callback_calc_pose)
        self.execute_pose_service = rospy.Service("/execute_pose", SetBool, self.execute_pose_goal)
        self.shutdown_subscriber = rospy.Subscriber("/shutdown_gui", Bool, self.shutdown)
        self.plan_cartesian_service = rospy.Service("/plan_cartesian", CalculateJoints, self.plan_cartesian)
        self.action_server_execute = actionlib.SimpleActionServer("execute_action", ExecuteDesiredPoseAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server_execute.start()
        self.scale_m = 0.001
        self.roll_input = 0


    def execute_cb(self,goal):
        success = True
        feedback_action = ExecuteDesiredPoseFeedback()
        result_action = ExecuteDesiredPoseResult()
        rate = rospy.Rate(100)

        if goal.command:
            feedback_action.feedback = "Executing....."
            plan = self.move_group.go(wait=True)
            self.action_server_execute.publish_feedback(feedback_action)
            self.move_group.stop()
            feedback_action.feedback = "Finished Executing....."
            self.action_server_execute.publish_feedback(feedback_action)
            self.move_group.clear_pose_targets()
            current_pose = self.move_group.get_current_pose().pose
            all_close(self.pose_goal.pose, current_pose, 0.01)

            if success:
                result_action.result = "Execution Successfuly"
                self.action_server_execute.set_succeeded(result_action)


    def plan_cartesian(self, req):
        waypoints = []

        scale=1
        wpose = self.move_group.get_current_pose(end_effector_link="eff").pose

        wpose.position.x += scale * req.x_input*self.scale_m
        wpose.position.y += scale * req.y_input*self.scale_m
        wpose.position.z += scale * req.z_input*self.scale_m

        orientation_list = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        roll += scale * math.radians(self.roll_input)
        pitch += scale * math.radians(req.pitch_input)
        yaw += scale * math.radians(req.yaw_input)*(-1)

        quaternion = tf.transformations.quaternion_from_euler(float(roll) ,float(pitch) ,float(yaw))
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]

        waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0
        attempts = 0
        max_tries = 100
        success = False

        while fraction < 1.0 and attempts < max_tries:
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
            attempts += 1
            if fraction == 1.0:
                 success = True

        if success:
            my_scale=1000
            modified_coodinates = []
            modified_coodinates.append(round(my_scale*plan.joint_trajectory.points[-1].positions[1], 2))
            modified_coodinates.append(round(my_scale*plan.joint_trajectory.points[-1].positions[0],2))
            modified_coodinates.append(round(-my_scale*plan.joint_trajectory.points[-1].positions[2],2))
            modified_coodinates.append(round((-1)*math.degrees(plan.joint_trajectory.points[-1].positions[4]),2))
            modified_coodinates.append(0)


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

            quaternion = tf.transformations.quaternion_from_euler(math.radians(self.roll_input) ,math.radians(req.pitch_input) ,math.radians(req.yaw_input))
            self.pose_goal.pose.orientation.x = quaternion[0]
            self.pose_goal.pose.orientation.y = quaternion[1]
            self.pose_goal.pose.orientation.z = quaternion[2]
            self.pose_goal.pose.orientation.w = quaternion[3]
            self.pose_goal.pose.position.x = req.x_input*self.scale_m
            self.pose_goal.pose.position.y = req.y_input*self.scale_m
            self.pose_goal.pose.position.z = req.z_input*self.scale_m

            self.move_group.set_pose_target(self.pose_goal.pose)

            my_scale = 1000
            calc_plan = self.move_group.plan()
            #print("CALC_PLAN:",calc_plan)
            length = len(calc_plan[1].joint_trajectory.points)
            drehung = req.yaw_input//360.19

            if length:
                valid_plan = True

                for i in range(length):
                    my_x = round(my_scale*calc_plan[1].joint_trajectory.points[-2+i].positions[1], 2)
                    my_y = round(my_scale*calc_plan[1].joint_trajectory.points[-2+i].positions[0],2)
                    my_z = round(-my_scale*calc_plan[1].joint_trajectory.points[-2+i].positions[2],2)
                    if math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3])<0.19 and math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3])>0:
                        my_c = 0 + drehung*360
                    else:
                        my_c = round(360-math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[3]),2) + drehung*360

                    my_b = round((-1)*math.degrees(calc_plan[1].joint_trajectory.points[-2+i].positions[4]),2)


                    array = [my_x, my_y, my_z, my_b,my_c]

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
