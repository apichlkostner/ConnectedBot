cmake_minimum_required(VERSION 3.5.0)

project(connected_bot_app)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(OpenCV REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  actionlib_msgs
  genmsg
  actionlib
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action FILES photo.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES show_robot_camera_gui
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs actionlib_msgs
  DEPENDS
)

add_executable(show_robot_camera_gui
  src/mainwindow.ui
  src/mainwindow.cpp
  src/show_robot_camera_node.cpp
  include/mainwindow.h
  include/show_robot_camera_node.h
)

target_include_directories(show_robot_camera_gui
  PRIVATE
    ./include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(show_robot_camera_gui
  PUBLIC
    Qt5::Widgets Qt5::Core ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

target_compile_features(show_robot_camera_gui
  PRIVATE
    cxx_lambdas
    cxx_auto_type
    cxx_constexpr
    cxx_deleted_functions
    cxx_nullptr
    cxx_range_for
    cxx_rvalue_references
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_connected_bot_app.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
