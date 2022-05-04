cmake_minimum_required(VERSION 3.0.2)
project(knowledge_core)


find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  rostest
)

catkin_python_setup()

add_service_files(
  FILES
  Manage.srv
  Revise.srv
  Query.srv
  Sparql.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS bin/knowledge_core
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_ros.test)
endif()
