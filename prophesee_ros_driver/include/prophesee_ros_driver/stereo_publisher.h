/*******************************************************************
 * File : prophesee_ros_publisher.h                                *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef STEREO_PUBLISHER_H_
#define STEREO_ROS_PUBLISHER_H_

#include <sensor_msgs/CameraInfo.h>

#include <prophesee_driver.h>

#include "log_tone_mapper.h"

#include "ros/ros.h"

/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperStereoPublisher {
public:
    /// \brief Constructor
    PropheseeWrapperStereoPublisher();

    /// \brief Destructor
    ~PropheseeWrapperStereoPublisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing();////TODO////

private:

    ///
    /// \brief Opens the camera that is being passed
    /// \param camera_ reference to one of the camera object (left/right)
    /// \return Returns if camera opening was successful
    ///
    bool openCamera(Prophesee::Camera & camera_);

    /// \brief Publishes CD events
    void publishCDEvents(Prophesee::Camera & camera, ros::Publisher & publisher);

    /// \brief Publishes gray-level frames
    void publishGrayLevels(Prophesee::Camera & camera, ros::Publisher & publisher);

    /// \brief Publishes IMU events
    void publishIMUEvents(Prophesee::Camera & camera, ros::Publisher & publisher, const std::string camPos);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_left;
    ros::Publisher pub_info_right;

    /// \brief Publisher for CD events
    ros::Publisher pub_cd_events_left;
    ros::Publisher pub_cd_events_right;

    /// \brief Publisher for gray-level frame
    ros::Publisher pub_gl_frame_left;
    ros::Publisher pub_gl_frame_right;

    /// \brief Publisher for IMU events
    ros::Publisher pub_imu_events_left;
    ros::Publisher pub_imu_events_right;

    /// \brief Instance of Camera class for left camera
    ///
    /// Used to access data from the left camera. Serial
    /// numer to left/right are taken from ros parameter
    /// space
    Prophesee::Camera camera_left;

    /// \brief Instance of Camera class for right camera
    ///
    /// Used to access data from the right camera. Serial
    /// numer to left/right are taken from ros parameter
    /// space
    Prophesee::Camera camera_right;

    /// \brief Instance of LogToneMapper class
    ///
    /// Used to reconstract gray-levels from CD and EM data and apply tone mapping
    LogToneMapper tone_mapper_;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_left;
    sensor_msgs::CameraInfo cam_info_msg_right;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief left camera name in string format
    std::string camera_name_left;

    /// \brief Left camera serial in string format
    std::string left_camera_id;

    /// \brief right camera name in string format
    std::string camera_name_right;

    /// \brief Right camera serial in string format
    std::string right_camera_id;

    /// \brief Camera string time
    ros::Time start_timestamp_;

    /// \brief Maximum events rate, in kEv/s
    int max_event_rate_;

    /// \brief Grey-level rate, in fps
    int graylevel_rate_;

    bool master_left_;

    /// \brief If showing CD events
    bool publish_cd_;

    /// \brief If showing gray-level frames
    bool publish_graylevels_;

   /// \brief If showing IMU events
    bool publish_imu_;

    static constexpr double GRAVITY = 9.81; /** Mean gravity value at Earth surface [m/s^2] **/
};

#endif /* STEREO_PUBLISHER_H_ */
