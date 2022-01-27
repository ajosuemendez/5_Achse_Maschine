cmake_minimum_required(VERSION 3.1.3)
project(project_praktikum_moveit_config)

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
    scripts/execute_cartesian_goal.py
    scripts/execute_rectangle.py
    scripts/execute_rombo.py
    scripts/execute_circle.py
    scripts/execute_rotation.py
    scripts/print_hello.py
    scripts/ser_com.py
    scripts/ros_ser_com.py
    scripts/send_command.py
    scripts/cls_ros_ser_com.py
    scripts/rep_send_command.py
    scripts/listener_send_command.py
    scripts/homing.py
    scripts/homing_joints.py
    scripts/set_joints.py
    scripts/test_path_constraints.py
    scripts/cartesian_path.py
    scripts/save_gcode.py
    scripts/joint_space_planning.py
    scripts/simple_gui.py
    scripts/data_exchange.py
    scripts/serviceTest.py
    scripts/dold_gui_2.py
    scripts/simpleActionClient.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
