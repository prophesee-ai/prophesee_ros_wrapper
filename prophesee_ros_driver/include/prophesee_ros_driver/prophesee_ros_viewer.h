/*******************************************************************
 * File : prophesee_ros_viewer.h                                   *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef PROPHESEE_ROS_VIEWER_H_
#define PROPHESEE_ROS_VIEWER_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>

#include "cd_frame_generator.h"

/// \brief Main class ROS listener and viewer
///
/// Listens ROS topics publishing data from Prophesee cameras and visualizes them on a screen
class PropheseeWrapperViewer {
public:
    /// \brief Constructor
    PropheseeWrapperViewer();

    /// \brief Destructor
    ~PropheseeWrapperViewer();

    /// \brief Shows currently available CD data
    void showData();

    /// \brief Checks if the frame generator class is initialized or not
    ///
    /// @return true if initialized and false otherwise
    bool isInitialized();

private:
    /// \brief Callback triggered when data are received from the camera info topic
    ///
    /// It gets width and height of the sensor and calls init() function
    ///
    /// @param msg : ROS message with the camera info
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /// \brief Initializes the frame generators and the displayers
    ///
    /// @param sensor_width : Width of the sensor
    /// @param sensor_height : Height of the sensor
    ///
    /// It initializes CD frame generator with the sensor's width and height.
    /// It also creates the displayers.
    bool init(const unsigned int &sensor_width, const unsigned int &sensor_height);

    /// \brief Creates a displayer
    ///
    /// @param window_name The name of the displayer window
    /// @param sensor_width : Width of the window
    /// @param sensor_height : Height of the window
    /// @param shift_x : Position (x shift) of the window
    /// @param shift_y : Position (y shift) of the window
    void create_window(const std::string &window_name, const unsigned int &sensor_width,
                       const unsigned int &sensor_height, const int &shift_x = 0, const int &shift_y = 0);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Subscriber to the camera info topic
    ros::Subscriber sub_cam_info_;

    /// \brief Subscriber for CD events topic
    ros::Subscriber sub_cd_events_;

    /// \brief Instance of CDFrameGenerator class that generates a frame from CD events
    CDFrameGenerator cd_frame_generator_;

    /// \brief Window name for visualizing CD events
    std::string cd_window_name_;

    /// \brief Display accumulation time

    /// The time interval to display events up to the current time, in us
    int display_acc_time_;

    /// \brief If the frame generators are initialized with teh sensor width and height
    bool initialized_;

    /// \brief  If visualizing CD events
    bool show_cd_ = true;

};

#endif /* PROPHESEE_ROS_VIEWER_H_ */
