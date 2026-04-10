cmake_minimum_required(VERSION 3.8) # require minimum cmake version
project(dev_cpp) # define package project name

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") # add extra warnings for gnu and clang
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-overloaded-virtual) # enable common warning flags
endif() # close compiler check

if(NOT CMAKE_CXX_STANDARD) # set default language standard when not already set
  set(CMAKE_CXX_STANDARD 14) # use c plus plus 14
endif() # close language standard check

find_package(ament_cmake REQUIRED) # load ament cmake build system
find_package(rclcpp REQUIRED) # load ros2 cpp client library
find_package(std_msgs REQUIRED) # load standard message package
find_package(sensor_msgs REQUIRED) # load sensor message package
find_package(geometry_msgs REQUIRED) # load geometry message package
find_package(ament_index_cpp REQUIRED) # load package share lookup library
find_package(OpenCV REQUIRED) # load opencv for yaml file storage

add_executable(template_camera src/template_camera.cpp) # build template camera executable
ament_target_dependencies(template_camera rclcpp sensor_msgs ament_index_cpp) # declare ros dependencies for template camera
target_link_libraries(template_camera ${OpenCV_LIBS}) # link opencv libraries for yaml writing

add_executable(template_object src/template_object.cpp) # build template object executable
ament_target_dependencies(template_object rclcpp sensor_msgs geometry_msgs) # declare ros dependencies for template object

add_executable(template_robot src/template_robot.cpp) # build template robot executable
ament_target_dependencies(template_robot rclcpp std_msgs geometry_msgs) # declare ros dependencies for template robot

add_executable(template_displays src/template_displays.cpp) # build template displays executable
ament_target_dependencies(template_displays rclcpp sensor_msgs std_msgs geometry_msgs) # declare ros dependencies for template displays

add_executable(template_offset src/template_offset.cpp) # build template offset executable
ament_target_dependencies(template_offset rclcpp std_msgs geometry_msgs) # declare ros dependencies for template offset

install( # install built executables into the package
  TARGETS # begin executable install list
    template_camera # install template camera executable
    template_object # install template object executable
    template_robot # install template robot executable
    template_displays # install template displays executable
    template_offset # install template offset executable
  DESTINATION lib/${PROJECT_NAME}) # install executables under package lib directory

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config) # install config directory into package share
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) # install launch directory into package share

ament_package() # finalize ament package metadata
