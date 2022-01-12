import rospy
from kivymd.app import MDApp
from kivymd.uix.dialog import MDDialog
from kivy.lang import Builder
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from project_praktikum_moveit_config.srv import CalculateJoints
from kivymd.theming import ThemeManager

class MainApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.path = '/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/guis/gui_nav.kv'
        self.screen = Builder.load_file(self.path)
        self.theme_cls = ThemeManager()

    def build(self):
        self.theme_cls.primary_palette = "Gray"
        return self.screen

if __name__ =='__main__':
    MainApp().run()
