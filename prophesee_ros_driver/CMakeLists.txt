cmake_minimum_required(VERSION 2.8.3)
project(prophesee_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


execute_process(COMMAND lsb_release -cs OUTPUT_VARIABLE RELEASE_CODENAME OUTPUT_STRIP_TRAILING_WHITESPACE)
if ("${RELEASE_CODENAME}" STREQUAL  "bionic")
	message ("Ubuntu 18.04 found")
	set(OpenCV_DIR /usr/share/OpenCV)
elseif ("${RELEASE_CODENAME}" STREQUAL  "xenial")
	message ("UBUNTU 16.04 found")
	set(OpenCV_DIR /usr/local/share/OpenCV)
endif ()

find_package(catkin REQUIRED COMPONENTS
        OpenCV
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        prophesee_event_msgs
        genmsg
        message_generation
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
    prophesee_driver
)

add_dependencies(
    prophesee_ros_publisher
    prophesee_event_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
    )

# Stereo Publisher
add_executable(stereo_publisher
    src/stereo_publisher.cpp
    src/log_tone_mapper.cpp)

target_link_libraries(
    stereo_publisher
    ${catkin_LIBRARIES}
    prophesee_driver
)

add_dependencies(
    stereo_publisher
    prophesee_event_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
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
    prophesee_driver
)

add_dependencies(
    prophesee_ros_viewer
    prophesee_event_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
    )
