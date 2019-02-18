/*******************************************************************
 * File : prophesee_ros_publisher.cpp                              *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <prophesee_event_msgs/PropheseeEvent.h>
#include <prophesee_event_msgs/PropheseeEventBuffer.h>

#include "prophesee_ros_publisher.h"

PropheseeWrapperPublisher::PropheseeWrapperPublisher():
    nh_("~"),
    biases_file_(""),
    max_event_rate_(6000),
    graylevel_rate_(30)
{
    std::string camera_name("camera");

    // Load Parameters
    nh_.getParam("camera_name", camera_name);
    nh_.getParam("publish_cd", publish_cd_);
    nh_.getParam("publish_graylevels", publish_graylevels_);
    nh_.getParam("bias_file", biases_file_);
    nh_.getParam("max_event_rate", max_event_rate_);
    nh_.getParam("graylevel_frame_rate", graylevel_rate_);

    const std::string topic_cam_info = "/prophesee/" + camera_name + "/camera_info";
    const std::string topic_cd_event_buffer = "/prophesee/" + camera_name + "/cd_events_buffer";
    const std::string topic_gl_frame = "/prophesee/" + camera_name + "/graylevel_image";

    pub_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_cam_info, 1);

    if (publish_cd_)
        pub_cd_events_ = nh_.advertise<prophesee_event_msgs::PropheseeEventBuffer>(topic_cd_event_buffer, 500);

    if (publish_graylevels_)
        pub_gl_frame_ = nh_.advertise<cv_bridge::CvImage>(topic_gl_frame, 1);

    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_INFO("Trying to open camera...");
    }

    // Set the maximum event rate, in kEv/s
    // Decreasing the max events rate will decrease the latency and also decrease the number of published events
    // Increasing the max events rate will increase the number of published events and also increase the latency
    if (!camera_.set_max_event_rate_limit(max_event_rate_)) {
         ROS_WARN("Failed to set the maximum event rate");
    }

    // Add camera runtime error callback
    camera_.add_runtime_error_callback(
                [](const Prophesee::CameraException &e) { ROS_WARN("%s", e.what()); });

    // Get the sensor config
    Prophesee::CameraConfiguration config = camera_.get_camera_configuration();
    auto &geometry = camera_.geometry();
    ROS_INFO("[CONF] Width:%i, Height:%i", geometry.width(), geometry.height());
    ROS_INFO("[CONF] Max event rate, in kEv/s: %u", config.max_drop_rate_limit_kEv_s);
    ROS_INFO("[CONF] Serial number: %s", config.serial_number.c_str());

    // Publish camera info message
    cam_info_msg_.width = geometry.width();
    cam_info_msg_.height = geometry.height();
    cam_info_msg_.header.frame_id = "PropheseeCamera_optical_frame";
}

PropheseeWrapperPublisher::~PropheseeWrapperPublisher() {
    camera_.stop();

    nh_.shutdown();
}

bool PropheseeWrapperPublisher::openCamera() {
    bool camera_is_opened = false;

    // Initialize the camera instance
    try {
        camera_ = Prophesee::Camera::from_first_available();
        if (!biases_file_.empty()) {
            camera_.biases().set_from_file(biases_file_);
        }
        camera_is_opened = true;
    } catch (Prophesee::CameraException &e) {
        ROS_WARN("%s", e.what());
    }
    return camera_is_opened;
}

void PropheseeWrapperPublisher::startPublishing() {
    camera_.start();

    if (publish_cd_)
        publishCDEvents();

    if (publish_graylevels_)
        publishGrayLevels();

    ros::Rate loop_rate(5);
    while(ros::ok()) {
        if (pub_info_.getNumSubscribers() > 0) {
            cam_info_msg_.header.stamp = ros::Time::now();
            pub_info_.publish(cam_info_msg_);
        }
        loop_rate.sleep();
    }
}

void PropheseeWrapperPublisher::publishCDEvents() {
    // Initialize and publish a buffer of CD events
    try {
        Prophesee::CallbackId cd_callback = camera_.cd().add_callback(
            [this](const Prophesee::EventCD *ev_begin, const Prophesee::EventCD *ev_end) {
                // Check the number of subscribers to the topic
                if (pub_cd_events_.getNumSubscribers() <= 0)
                    return;

                if (ev_begin < ev_end) {
                    // Define the message for a buffer of CD events
                    prophesee_event_msgs::PropheseeEventBuffer event_buffer_msg;
                    const unsigned int buffer_size = ev_end - ev_begin;
                    event_buffer_msg.events.resize(buffer_size);

                    // Add events to the message
                    auto buffer_it = event_buffer_msg.events.begin();
                    for (const Prophesee::EventCD *it = ev_begin; it != ev_end; ++it, ++buffer_it) {
                        prophesee_event_msgs::PropheseeEvent &event = *buffer_it;
                        event.x = it->x;
                        event.y = it->y;
                        event.p = it->p;
                        event.t = it->t;
                    }

                    // Publish the message
                    pub_cd_events_.publish(event_buffer_msg);

                    ROS_DEBUG("CD data available, buffer size: %d at time: %llu", buffer_size, ev_begin->t);
                }
            });
    } catch (Prophesee::CameraException &e) {
        ROS_WARN("%s", e.what());
        publish_cd_ = false;
    }
}

void PropheseeWrapperPublisher::publishGrayLevels() {
    // Initialize and publish a gray-level frame
    try {
        camera_.set_exposure_frame_callback(graylevel_rate_, [this](Prophesee::timestamp t, const cv::Mat &f) {
            // Check the number of subscribers to the topic
            if (pub_gl_frame_.getNumSubscribers() <= 0)
                return;

            // Define the message for the gray level frame
            cv_bridge::CvImage gl_frame_msg;
            gl_frame_msg.header.stamp = ros::Time::now();
            gl_frame_msg.header.frame_id = "PropheseeCamera_optical_frame";
            gl_frame_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            gl_frame_msg.image = tone_mapper_(f);

            // Publish the message
            pub_gl_frame_.publish(gl_frame_msg);

            ROS_DEBUG("Graylevel data are available");
        });
    } catch (Prophesee::CameraException &e) {
        if (e.code().value() & Prophesee::CameraErrorCode::UnsupportedFeature)
            publish_graylevels_ = false;
        else {
            ROS_WARN("%s", e.what());
            publish_graylevels_ = false;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prophesee_ros_publisher");

    PropheseeWrapperPublisher wp;
    wp.startPublishing();

    ros::shutdown();

    return 0;
}
