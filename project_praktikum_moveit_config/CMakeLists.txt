cmake_minimum_required(VERSION 3.1.3)
project(project_praktikum_moveit_config)

find_package(catkin REQUIRED)

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
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
