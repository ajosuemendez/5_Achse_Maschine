cmake_minimum_required(VERSION 3.1.3)
project(dummy_robot_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib_msgs
)
add_service_files(
  FILES
  CalculateJoints.srv
  SendCommand.srv
)

add_action_files(DIRECTORY action
  FILES
  SerialCommunication.action
  ExecuteDesiredPose.action
  ExecuteCartesianDesiredPose.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)


catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
    scripts/execute_pose_goal.py
    scripts/dold_gui_6.py
    scripts/dold_gui_7.py
    scripts/dold_gui_8.py
    scripts/dold_gui_9.py
    scripts/cls_ros_ser_com.py
    scripts/get_joints.py
    scripts/save_gcode.py
    scripts/pilz_commander_planning.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
