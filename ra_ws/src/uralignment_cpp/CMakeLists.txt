cmake_minimum_required(VERSION 3.8)
project(uralignment_cpp)

# Compiler options for generating warnings if something goes wrong
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-overloaded-virtual) 
  # Wall enables all common warnings
  # Wextra (Wall extra) enables additional warnings
  # Wpendantic warns about non-standard code
  # Wno-overloaded-virtual suppresses warnings about overloaded virtual functions
endif()

# Find required packages:
find_package(VISP REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED) # Build system for building (compiling & linking) ROS2 packages
find_package(rclcpp REQUIRED) # Core ROS2 client library for C++ for Nodes & Topics
find_package(std_msgs REQUIRED) # Provide standard message types (strings, integers, etc) for Topics
find_package(sensor_msgs REQUIRED) # Provides predefined message types for sensor data
find_package(OpenCV REQUIRED) # OpenCV Library (Not a Dependency defined in package.xml since it's downloaded to the OS)
find_package(cv_bridge REQUIRED) # ROS2 library to convert ROS image messages to/from OpenCV images
find_package(ament_index_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_rtde REQUIRED CONFIG)

# Additional required directories:
include_directories(
  include
  /opt/ros/jazzy/include # to 'include' libraries and headers
  include # Tell CMake where to find package headers (.h files)
  ${VISP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

# demo_servo #######################################################
add_executable(demo_servo src/demo_servo.cpp) # Add to build process
target_link_libraries(demo_servo ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(demo_servo rclcpp std_msgs sensor_msgs image_transport cv_bridge ament_index_cpp)
####################################################################

# d435i_camera node ################################################
add_executable(d435i_camera src/d435i_camera.cpp)
target_include_directories(d435i_camera PRIVATE include)
target_link_libraries(d435i_camera ${OpenCV_LIBS} realsense2)
ament_target_dependencies(d435i_camera rclcpp sensor_msgs std_msgs image_transport cv_bridge)
####################################################################

# image_subscriber #################################################
add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV)
####################################################################

# apriltags ########################################################
add_executable(apriltags src/apriltags.cpp) # Add to build process
target_link_libraries(apriltags ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(apriltags rclcpp geometry_msgs sensor_msgs tf2 image_transport cv_bridge ament_index_cpp OpenCV)
####################################################################

# ur_e_series ######################################################
add_executable(ur_e_series src/ur_e_series.cpp) # Add to build process
target_link_libraries(ur_e_series ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(ur_e_series rclcpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge ament_index_cpp)
####################################################################

# displays #########################################################
add_executable(displays src/displays.cpp) # Add to build process
target_link_libraries(displays ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(displays rclcpp std_msgs sensor_msgs geometry_msgs image_transport cv_bridge ament_index_cpp OpenCV)
####################################################################

# teleop_key ####################################################
add_executable(teleop_key src/teleop_key.cpp)
target_link_libraries(teleop_key PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})
#################################################################

# offset ####################################################
add_executable(offset src/offset.cpp)
ament_target_dependencies(offset rclcpp std_msgs geometry_msgs)
#################################################################

# wrench ####################################################
add_executable(wrench src/wrench.cpp)
ament_target_dependencies(wrench rclcpp geometry_msgs)
target_link_libraries(wrench ur_rtde::rtde)
#################################################################

# tracker ##########################################################
add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker yaml-cpp ${OpenCV_LIBS})
ament_target_dependencies(tracker rclcpp std_msgs sensor_msgs geometry_msgs ament_index_cpp)
target_compile_features(tracker PUBLIC cxx_std_17) 
####################################################################



install(TARGETS
  demo_servo
  d435i_camera
  image_subscriber
  apriltags
  ur_e_series
  displays
  teleop_key
  offset
  wrench
  tracker
  DESTINATION lib/${PROJECT_NAME})

# Installs the built executables into ROS2 directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_package() # Declares the Package for ROS2
