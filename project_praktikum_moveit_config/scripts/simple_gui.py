#!/usr/bin/env python3.8
import rospy
from kivymd.app import MDApp
from kivymd.uix.dialog import MDDialog
from kivy.lang import Builder
from std_msgs.msg import Bool
from project_praktikum_moveit_config.srv import CalculateJoints



##################################################################################
##################################################################################
##################################################################################
class TutorialApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.path = '/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/guis/ros_gui.kv'
        self.screen = Builder.load_file(self.path)
        self.dialog = None



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
        #my calc
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


        roll = 0

        #send this data to a service
        rospy.wait_for_service('/calc_pose')
        print("SERVICE FOUND")

        service_conn = rospy.ServiceProxy('/calc_pose', CalculateJoints)
        print("SERVICE CONNECTED")
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
        #self.root.ids.calc_pos.text = ""
        pass

    def gcode(self):
        #self.root.ids.calc_pos.text = ""
        #my calc
        pass


if __name__ =='__main__':
    rospy.init_node('simple_gui', anonymous=True)

    pub = rospy.Publisher("/button", Bool, queue_size=1)

    TutorialApp().run()
