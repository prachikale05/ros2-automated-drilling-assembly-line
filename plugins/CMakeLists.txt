cmake_minimum_required(VERSION 3.10)
project(gripper_hold_plugin)

find_package(gazebo REQUIRED)

add_library(gripper_hold_plugin SHARED gripper_hold_plugin.cpp)

target_include_directories(gripper_hold_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gripper_hold_plugin ${GAZEBO_LIBRARIES})
target_compile_options(gripper_hold_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

install(TARGETS gripper_hold_plugin
  LIBRARY DESTINATION lib
)
