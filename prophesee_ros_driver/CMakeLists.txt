cmake_minimum_required(VERSION 2.8.3)
project(prophesee_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
        OpenCV
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        prophesee_event_msgs
)

find_package(prophesee_driver REQUIRED)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PROPHESEE_DRIVER_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

# Prophesee Publisher
add_executable(prophesee_ros_publisher
    src/prophesee_ros_publisher.cpp
    src/log_tone_mapper.cpp
)

target_link_libraries(
    prophesee_ros_publisher
    ${catkin_LIBRARIES}
    ${PROPHESEE_DRIVER_LIBRARIES}
)

# Prophesee Viewer
add_executable(prophesee_ros_viewer
    src/prophesee_ros_viewer.cpp
    src/cd_frame_generator.cpp
    src/log_tone_mapper.cpp
)

target_link_libraries(
    prophesee_ros_viewer
    ${catkin_LIBRARIES}
    ${PROPHESEE_DRIVER_LIBRARIES}
)
