cmake_minimum_required(VERSION 3.0.2)
project(minimalkb)


find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  Revise.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS bin/minimalkb
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
