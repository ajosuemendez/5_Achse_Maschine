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
import re
import numpy
from std_srvs.srv import SetBool



myatan = lambda x,y: numpy.pi*(1.0-0.5*(1+numpy.sign(x))*(1-numpy.sign(y**2))\
         -0.25*(2+numpy.sign(x))*numpy.sign(y))\
         -numpy.sign(x*y)*numpy.arctan((numpy.abs(x)-numpy.abs(y))/(numpy.abs(x)+numpy.abs(y)))

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
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


def call_bool_service(service):
    # Wait for the service server to be running
    rospy.wait_for_service(service)
    print("SERVICE FOUND")
    # Create the connection to the service
    service_conn = rospy.ServiceProxy(service, SetBool)
    print("SERVICE CONNECTED")
    try:
        request = SetBool()
        request.data = True
        response = service_conn(request.data)
        print(response)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
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
        #self.upright_constraints = moveit_msgs.msg.Constraints()
        #self.upright_constraints.name = "upright"

    def go_to_pose_goal(self, roll_input, pitch_input, yaw_input, x_input, y_input, z_input):

        scale_m = 0.001
        pose_goal = geometry_msgs.msg.PoseStamped()


        quaternion = tf.transformations.quaternion_from_euler(math.radians(roll_input) ,math.radians(pitch_input) ,math.radians(yaw_input))
        pose_goal.pose.orientation.x = quaternion[0]
        pose_goal.pose.orientation.y = quaternion[1]
        pose_goal.pose.orientation.z = quaternion[2]
        pose_goal.pose.orientation.w = quaternion[3]
        pose_goal.pose.position.x = x_input*scale_m
        pose_goal.pose.position.y = y_input*scale_m
        pose_goal.pose.position.z = z_input*scale_m

        #self.init_upright_path_constraints(pose_goal)


        self.move_group.set_pose_target(pose_goal.pose)

        plan = self.move_group.go(wait=True)



        self.move_group.stop()

        call_bool_service('/save_gcode_point')

        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        #my_quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
        #euler = tf.transformations.euler_from_quaternion(my_quat)

        #return all_close(pose_goal.pose, current_pose, 0.01)

    #def init_upright_path_constraints(self,pose_goal):


        #orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        #orientation_constraint.header.frame_id = pose_goal.header.frame_id
        #orientation_constraint.link_name = self.eef_link
        #orientation_constraint.orientation = pose_goal.pose.orientation
        #orientation_constraint.absolute_x_axis_tolerance = 3.6
        #orientation_constraint.absolute_y_axis_tolerance = 3.6
        #orientation_constraint.absolute_z_axis_tolerance = 3.6

        #orientation_constraint.weight = 1
        #self.upright_constraints.orientation_constraints.append(orientation_constraint)
        #self.move_group.set_path_constraints(self.upright_constraints)

    def go_to_joint_state(self, degree):

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[3] = degree

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        call_bool_service('/save_gcode_point')

        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def plan_cartesian_path(self, x_input, y_input, z_input, r_input, p_input, yaw_input):

        waypoints = []

        scale=1
        scale_m = 0.001

        wpose = self.move_group.get_current_pose(end_effector_link="eff").pose

        #wpose.position.z += scale * 0.1  # First move up (z)
        #wpose.position.y -= scale * 0.05 # and sideways (y)
        wpose.position.x += scale * (float(x_input)*scale_m)
        wpose.position.y += scale * (float(y_input)*scale_m)
        wpose.position.z += scale * (float(z_input)*scale_m)

        orientation_list = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        roll += scale * math.radians(float(r_input))
        pitch += scale * math.radians(float(p_input))
        yaw += scale * math.radians(float(yaw_input))*(-1)

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

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction, success, attempts


    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

def main():
    rospy.sleep(1.0) #in order to give the savegcode node time to initialize
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        lines = []
        full_list = []

        with open("/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/scripts/5d_samples.txt") as f:
            lines= f.readlines()

        #x_input = 600
        #y_input = 600
        #z_input = 550
        #roll_input = 0
        #pitch_input = i
        #yaw_input = 0

        #for i in range(0,-92, -2):
        #for i in range(0,-91,-2):
        #for i in range(1):
            #roll_input = input("Please enter the Roll in deg: ")
            #pitch_input = input("Please enter the Pitch in deg: ")
            #yaw_input = input("Please enter the Yaw in deg: ")
            #x_input = input("Please enter the x (between 174mm-1234mm) in mm: ")
            #y_input = input("Please enter the y (between 15mm-1515mm) in mm: ")
            #z_input = input("Please enter the z (between 180mm-630mm)in mm: ")
            #x_input = 600
            #y_input = 600
            #z_input = 550
            #roll_input = 0
            #pitch_input = i
            #yaw_input = 0
            #tutorial.go_to_pose_goal(roll_input, pitch_input, yaw_input, x_input, y_input, z_input)

        for line in lines:
            pattern = re.compile(r'([.-]|)\d*\.\d*')
            matches = pattern.finditer(str(line))
            tcp_coordinates = [round(float(match.group(0)),2) for match in matches]
            full_list.append(tcp_coordinates)


        coordinates = [(round(math.degrees(myatan(elem[3], elem[4])), 2), round(math.degrees(math.atan((math.sqrt( abs((elem[3])**2) + abs((elem[4])**2)))/elem[5])),2)) for elem in full_list]

        start_time = rospy.Duration(0)
        old_secs = start_time.to_sec()

        ERROR_FLAG = False

        for i in range(len(coordinates)):
            full_list[i].append(coordinates[i][0])
            full_list[i].append(coordinates[i][1])

            new_time = rospy.Time.now()
            new_secs = new_time.to_sec()
            dif = new_secs - old_secs
            old_secs = new_secs

            if i>=1:
                if abs(full_list[i][6]- full_list[i-1][6])>300:
                    m = math.tan(math.radians(full_list[i][7])) #steigung
                    x_deviation = 50 #in mm
                    z_deviation = m*x_deviation # linear equation in mm

                    cartesian_plan, fraction, success, attempts = tutorial.plan_cartesian_path(x_input=x_deviation, y_input=0, z_input=z_deviation, r_input=0, p_input=0, yaw_input=0)

                    if success:
                        print(f"Completed Trajectory Planned And To Be Executed After {attempts} Attempts")
                        tutorial.execute_plan(cartesian_plan)
                        call_bool_service('/save_gcode_point')
                        #determinating the direction (from 0 to 360 or 360 to 0)
                        if (full_list[i][6] - full_list[i-1][6]) < 0:
                            #positive then rotate the other way (go to 0)
                            print("going to 360 turning should be executed")
                            tutorial.go_to_joint_state(359.9)

                        else:
                            #negative then rotate the other way (go to 360)
                            print("going to 0 turning should be executed")
                            tutorial.go_to_joint_state(0.0001)

                        # cartesian_plan2, fraction2, success2, attempts2 = tutorial.plan_cartesian_path(x_input=-x_deviation, y_input=0, z_input=-z_deviation, r_input=0, p_input=0, yaw_input=0)
                        # if success2:
                        #     print(f"Completed Trajectory Planned And To Be Executed After {attempts} Attempts")
                        #     tutorial.execute_plan(cartesian_plan) #return none
                        #     call_bool_service('/save_gcode_point')
                        # else:
                        #     print(f"No proper Cartesian Plan Found after {attempts} Attempts!")
                        #     ERROR_FLAG = True
                        #     print("Finishing the execution")
                        #     break
                    else:
                        print(f"No proper Cartesian Plan Found after {attempts} Attempts!")
                        ERROR_FLAG = True
                        print("Finishing the execution")
                        break

            pos_offsets = [600,600,500] #corresponding x,y,z offset
            abs_pos = [pos_offsets[k] + full_list[i][k] for k in range(len(pos_offsets))]

            print(f"[time dif: {dif} ]to be executed x: {abs_pos[0]} y: {abs_pos[1]} z: {abs_pos[2]} yaw: {full_list[i][6]} pitch: {full_list[i][7]}" )

            #print("to be executed: ", tcp_coordinates)
            #tutorial.go_to_pose_goal(x_input = tcp_coordinates[0],
            #                         y_input = tcp_coordinates[1],
            #                         z_input = tcp_coordinates[2],
            #                         roll_input = tcp_coordinates[3],
            #                         pitch_input = tcp_coordinates[4],
            #                         yaw_input = tcp_coordinates[5])
            tutorial.go_to_pose_goal(x_input = abs_pos[0],
                                     y_input = abs_pos[1],
                                     z_input = abs_pos[2],
                                     roll_input = 0.00,
                                     pitch_input = full_list[i][7],
                                     yaw_input = full_list[i][6])
        if not ERROR_FLAG:
            call_bool_service('/save_gcode')
            call_bool_service('/show_max_min_pos')
            call_bool_service('/save_all_gcode_points')

        total_time = rospy.Time.now().to_sec() - start_time.to_sec()
        print("TOTAL TIME in sec:", total_time )

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
