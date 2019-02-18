/*******************************************************************
 * File : prophesee_ros_publisher.h                                *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef PROPHESEE_ROS_PUBLISHER_H_
#define PROPHESEE_ROS_PUBLISHER_H_

#include <sensor_msgs/CameraInfo.h>

#include <prophesee_driver.h>

#include "log_tone_mapper.h"

/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperPublisher {
public:
    /// \brief Constructor
    PropheseeWrapperPublisher();

    /// \brief Destructor
    ~PropheseeWrapperPublisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing();

private:

    /// \brief Opens the camera
    bool openCamera();

    /// \brief Publishes CD events
    void publishCDEvents();

    /// \brief Publishes gray-level frames
    void publishGrayLevels();

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_;

    /// \brief Publisher for CD events
    ros::Publisher pub_cd_events_;

    /// \brief Publisher for gray-level frame
    ros::Publisher pub_gl_frame_;

    /// \brief Instance of Camera class
    ///
    /// Used to access data from a camera
    Prophesee::Camera camera_;

    /// \brief Instance of LogToneMapper class
    ///
    /// Used to reconstract gray-levels from CD and EM data and apply tone mapping
    LogToneMapper tone_mapper_;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief Maximum events rate, in kEv/s
    int max_event_rate_;

    /// \brief Grey-level rate, in fps
    int graylevel_rate_;

    /// \brief If showing CD events
    bool publish_cd_;

    /// \brief If showing gray-level frames
    bool publish_graylevels_;
};

#endif /* PROPHESEE_ROS_PUBLISHER_H_ */
