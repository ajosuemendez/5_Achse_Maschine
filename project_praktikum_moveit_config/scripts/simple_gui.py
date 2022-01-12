#!/usr/bin/env python3.8
import rospy
from kivymd.app import MDApp
from kivymd.uix.dialog import MDDialog
from kivy.lang import Builder
from plyer import filechooser
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from project_praktikum_moveit_config.srv import CalculateJoints
import re
import math
import numpy

class Finisher():
    def __init__(self):
        self.pub = rospy.Publisher("/shutdown_gui", Bool, queue_size=1)
        self.msg = False

    def clean_shutdown(self):
        self.msg = True
        self.pub.publish(self.msg)

class GuiApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.path = '/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/guis/ros_gui.kv'
        self.screen = Builder.load_file(self.path)
        self.dialog = None
        self.calculated_plan = False
        self.full_list = []

    def build(self):
        self.theme_cls.primary_palette = "Gray"
        return self.screen

    def show_alert_dialog(self,message):
        if not self.dialog:
            self.dialog = MDDialog(
                text=message,
                radius=[20, 7, 20, 7],
            )
        self.dialog.open()

    def calculate(self, *args):
        try:
            x = float(self.screen.ids.x_coordinate.text)
            y = float(self.screen.ids.y_coordinate.text)
            z = float(self.screen.ids.z_coordinate.text)
            pitch = float(self.screen.ids.pitch_coordinate.text)
            yaw = float(self.screen.ids.yaw_coordinate.text)
        except ValueError:
            self.show_alert_dialog("please make sure to set all the values and valid format")
            self.dialog = None
            return None

        rospy.wait_for_service('/calc_pose')
        service_conn = rospy.ServiceProxy('/calc_pose', CalculateJoints)

        try:
            request = CalculateJoints()
            request.x_input = x
            request.y_input = y
            request.z_input = z
            request.pitch_input = pitch
            request.yaw_input = yaw
            response = service_conn(request.x_input, request.y_input, request.z_input, request.pitch_input, request.yaw_input)
            #print(response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        if not response.success:
            #display the data received from the service
            #current_joints
            self.screen.ids.current_joint_x.text = str(response.joints[0])
            self.screen.ids.current_joint_y.text = str(response.joints[1])
            self.screen.ids.current_joint_z.text = str(response.joints[2])
            self.screen.ids.current_joint_b.text = str(response.joints[3])
            self.screen.ids.current_joint_c.text = str(response.joints[4])
            #calculated joints
            self.screen.ids.calc_joint_x.text = str(response.joints[5])
            self.screen.ids.calc_joint_y.text = str(response.joints[6])
            self.screen.ids.calc_joint_z.text = str(response.joints[7])
            self.screen.ids.calc_joint_b.text = str(response.joints[8])
            self.screen.ids.calc_joint_c.text = str(response.joints[9])

            self.calculated_plan = True
        else:
            self.show_alert_dialog("No Motion Plan Found")
            self.dialog = None
            return None

    def clear(self):
        self.screen.ids.x_coordinate.text = ""
        self.screen.ids.y_coordinate.text = ""
        self.screen.ids.z_coordinate.text = ""
        self.screen.ids.pitch_coordinate.text = ""
        self.screen.ids.yaw_coordinate.text = ""

    def execute(self):
        if self.calculated_plan:
            rospy.wait_for_service('/execute_pose')
            print("SERVICE FOUND")

            service_conn = rospy.ServiceProxy('/execute_pose', SetBool)
            print("SERVICE CONNECTED")
            try:
                request = SetBool()
                request.data = True
                response = service_conn(request.data)
                #print(response)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            self.calculated_plan = False

        else:
            self.show_alert_dialog("There is no calculated trajectory plan")
            self.dialog = None
            return None

    def file_chooser(self):
        filechooser.open_file(on_selection=self.selected)

    def selected(self, selection):
        self.root.ids.dumper_file.text = selection[0]
        lines = []
        self.full_list = []

        with open(selection[0]) as f:
            lines= f.readlines()


        for line in lines:
            pattern = re.compile(r'([.-]|)\d*\.\d*')
            matches = pattern.finditer(str(line))
            tcp_coordinates = [round(float(match.group(0)),2) for match in matches]
            self.full_list.append(tcp_coordinates)

        myatan = lambda x,y: numpy.pi*(1.0-0.5*(1+numpy.sign(x))*(1-numpy.sign(y**2))\
                 -0.25*(2+numpy.sign(x))*numpy.sign(y))\
                 -numpy.sign(x*y)*numpy.arctan((numpy.abs(x)-numpy.abs(y))/(numpy.abs(x)+numpy.abs(y)))

        coordinates = [(float("{:.2f}".format(math.degrees(myatan(elem[3], elem[4])))), float("{:.2f}".format(math.degrees(math.atan(math.sqrt(math.fabs(math.pow(elem[3],2)) + math.fabs(math.pow(elem[4],2)))/elem[5]))))) for elem in self.full_list]


        for i in range(len(coordinates)):
            self.full_list[i].append(coordinates[i][0])
            self.full_list[i].append(coordinates[i][1])


    def generate_gcode(self):
        self.screen.ids.gcode_field.text = self.full_list



    def save_file(self):
        pass

    def gcode(self):
        pass

if __name__ =='__main__':
    rospy.init_node('simple_gui', anonymous=True)
    rospy.on_shutdown(Finisher().clean_shutdown)
    GuiApp().run()
