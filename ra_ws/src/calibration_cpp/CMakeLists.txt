cmake_minimum_required(VERSION 3.16)
project(calibration_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VISP REQUIRED)
find_package(ur_rtde REQUIRED CONFIG)

find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)

# Use the ROS Jazzy librealsense2
find_library(REALSENSE2_LIB
  NAMES realsense2
  PATHS /opt/ros/jazzy/lib/x86_64-linux-gnu
  REQUIRED
)
message(STATUS "Using librealsense2: ${REALSENSE2_LIB}")

list(REMOVE_ITEM VISP_LIBRARIES "/usr/lib/x86_64-linux-gnu/librealsense2.so")
list(REMOVE_ITEM VISP_LIBRARIES "/usr/lib/x86_64-linux-gnu/librealsense2.so.2.56")
list(REMOVE_ITEM VISP_LIBRARIES "/usr/lib/x86_64-linux-gnu/librealsense2.so.2.56.5")

include_directories(
  include
  /opt/ros/jazzy/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

# calib_utils library
add_library(calib_utils
  src/terminal_io.cpp
  src/transform_math.cpp
  src/yaml_io.cpp
)
ament_target_dependencies(calib_utils
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge ament_index_cpp
)
target_link_libraries(calib_utils ${OpenCV_LIBS})


# intrinsics_calib node #############################################################
add_executable(intrinsics_calib src/intrinsics_calib.cpp)
ament_target_dependencies(intrinsics_calib
  rclcpp sensor_msgs std_msgs cv_bridge ament_index_cpp
)
target_link_libraries(intrinsics_calib
  calib_utils
  ${OpenCV_LIBS}
  ${REALSENSE2_LIB}
)
target_compile_options(intrinsics_calib PRIVATE
  -Wno-deprecated-copy
  -Wno-missing-field-initializers
  -Wno-pessimizing-move
  -Wno-ignored-qualifiers
)
#####################################################################################


# rtde_state node ###################################################################
add_executable(rtde_state src/rtde_state.cpp)
ament_target_dependencies(rtde_state
  rclcpp geometry_msgs
)
target_link_libraries(rtde_state ur_rtde::rtde ${OpenCV_LIBS})
#####################################################################################


# camera_extrinsics_calib node ######################################################
add_executable(camera_extrinsics_calib src/camera_extrinsics_calib.cpp)
ament_target_dependencies(camera_extrinsics_calib
  rclcpp geometry_msgs
)
target_link_libraries(camera_extrinsics_calib
  calib_utils
  ${OpenCV_LIBS}
)
#####################################################################################


# tooling_extrinsics_calib node #####################################################
add_executable(tooling_extrinsics_calib src/tooling_extrinsics_calib.cpp)
ament_target_dependencies(tooling_extrinsics_calib
  rclcpp geometry_msgs ament_index_cpp
)
target_link_libraries(tooling_extrinsics_calib
  calib_utils
  ${OpenCV_LIBS}
)
####################################################################################


#apriltag_node##########################################################################
add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  tf2
  ament_index_cpp
)
target_link_libraries(apriltag_node
  ${OpenCV_LIBS}
  ${VISP_LIBRARIES}
)
install(TARGETS
  apriltag_node
  DESTINATION lib/${PROJECT_NAME}
)
#####################################################################################


# displays #########################################################
add_executable(displays_node src/displays_node.cpp)
target_link_libraries(displays_node ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${REALSENSE2_LIB})
ament_target_dependencies(displays_node rclcpp std_msgs sensor_msgs geometry_msgs image_transport cv_bridge ament_index_cpp OpenCV)
install(TARGETS
  displays_node
  DESTINATION lib/${PROJECT_NAME}
)
####################################################################


# checkerboard_node #########################################################
add_executable(checkerboard_node src/checkerboard_node.cpp)
target_link_libraries(checkerboard_node ${VISP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${REALSENSE2_LIB})
ament_target_dependencies(checkerboard_node rclcpp std_msgs sensor_msgs geometry_msgs image_transport cv_bridge ament_index_cpp OpenCV tf2)
####################################################################


# pendant_tcp node ##########################################################
add_executable(pendant_tcp src/pendant_tcp.cpp)
ament_target_dependencies(pendant_tcp rclcpp)
target_link_libraries(pendant_tcp ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(pendant_tcp PRIVATE ${OpenCV_INCLUDE_DIRS})
#############################################################################




install(TARGETS
  pendant_tcp
  checkerboard_node
  calib_utils
  intrinsics_calib
  rtde_state
  camera_extrinsics_calib
  tooling_extrinsics_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()