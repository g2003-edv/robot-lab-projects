cmake_minimum_required(VERSION 3.0.2)
project(quadruped_gait)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

# Node script: installed as an executable.
catkin_install_python(PROGRAMS
  scripts/gait_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# kinematics.py is a library imported by the node, not a node of its own: it is
# installed as a plain file next to gait_controller.py so that
# `import kinematics` keeps working in the install space.
install(FILES scripts/kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
