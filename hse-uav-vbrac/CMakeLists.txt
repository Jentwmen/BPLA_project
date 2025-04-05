cmake_minimum_required(VERSION 2.8.3)
project(enoga)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  keyboard
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  enoga.msg
  follower.msg
  points.msg
  battery.msg
  formationMsg.msg
  algMsg.msg
  guiMas.msg
  groups.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
      CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(leader_node src/leader_node.cpp)
target_link_libraries(leader_node
   ${catkin_LIBRARIES}
)
add_dependencies(leader_node enoga_generate_messages_cpp)

add_executable(follower_enoga src/follower_node.cpp)
target_link_libraries(follower_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(follower_enoga enoga_generate_messages_cpp)

add_executable(follower_enoga2 src/follower_node2.cpp)
target_link_libraries(follower_enoga2
   ${catkin_LIBRARIES}
)
add_dependencies(follower_enoga2 enoga_generate_messages_cpp)

add_executable(commands_controller_enoga src/commands_controller.cpp)
target_link_libraries(commands_controller_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(commands_controller_enoga enoga_generate_messages_cpp)

add_executable(test_surf_enoga src/test_surf.cpp)
target_link_libraries(test_surf_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(test_surf_enoga enoga_generate_messages_cpp)

add_executable(participant_enoga src/participant_node.cpp src/uav.cpp src/formation.cpp)
target_link_libraries(participant_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(participant_enoga enoga_generate_messages_cpp)

add_executable(test_enoga src/unit_test.cpp src/uav.cpp src/formation.cpp)
target_link_libraries(test_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(test_enoga enoga_generate_messages_cpp)

add_executable(skforms_enoga src/skforms.cpp)
target_link_libraries(skforms_enoga
   ${catkin_LIBRARIES}
)
add_dependencies(skforms_enoga enoga_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/test_node.py scripts/camera_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#catkin_package(
#      CATKIN_DEPENDS roscpp std_msgs message_runtime
#)

#catkin_package()
