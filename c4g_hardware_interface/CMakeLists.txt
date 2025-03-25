cmake_minimum_required(VERSION 3.0.2)
project(c4g_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  c4g_open_driver
  hardware_interface
  controller_manager
  std_msgs
  sensor_msgs
  gripper_hardware_interface
  ros_extensions_utilities
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS c4g_open_driver hardware_interface controller_manager std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Specify hooks to source when the package is built
SET(MULTIPLE_ARMS OFF CACHE BOOL "Set to ON if you want to control multiple arms")
if(MULTIPLE_ARMS)
  catkin_add_env_hooks(setup
    SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

add_library(${PROJECT_NAME} src/c4g_hardware_interface.cpp)

add_executable(main_loop src/main_loop.cpp)
add_executable(demo_c4g_trajectory_planner demo/demo_c4g_trajectory_planner.cpp)

add_executable(main_loop_with_gripper src/main_loop_with_gripper.cpp)

target_link_libraries(
  main_loop
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  main_loop_with_gripper
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


target_link_libraries(
  demo_c4g_trajectory_planner
  ${catkin_LIBRARIES}
)
