cmake_minimum_required(VERSION 2.8.3)
project(lesson2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  turtlesim
  actionlib_msgs
  actionlib
)

## Generate actions in the 'action' folder
 add_action_files(DIRECTORY action FILES GoDestination.action)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lesson2
  CATKIN_DEPENDS geometry_msgs roscpp tf turtlesim
  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} include)

add_executable(turtle_pose src/turtle_pose.cpp)
target_link_libraries(turtle_pose ${catkin_LIBRARIES})

add_executable(turtle_controller src/turtle_controller.cpp)
target_link_libraries(turtle_controller ${catkin_LIBRARIES})


add_executable(GoDestination_client src/GoDestination_client.cpp)
target_link_libraries(GoDestination_client ${catkin_LIBRARIES})
add_dependencies(GoDestination_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(GoDestination_server src/GoDestination_server.cpp)
target_link_libraries(GoDestination_server ${catkin_LIBRARIES})
add_dependencies(GoDestination_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(pose_broadcaster src/rviz_pose.cpp)
target_link_libraries(pose_broadcaster ${catkin_LIBRARIES})

