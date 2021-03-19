/*******************************************************************
 * File : prophesee_ros_viewer.cpp                                 *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include "prophesee_ros_viewer.h"

#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif

typedef const boost::function<void(const prophesee_event_msgs::EventArray::ConstPtr &msgs)> callback;

PropheseeWrapperViewer::PropheseeWrapperViewer() :
    nh_("~"),
    cd_window_name_("CD Events"),
    display_acc_time_(5000),
    initialized_(false) {
    std::string camera_name("");

    // Load Parameters
    nh_.getParam("camera_name", camera_name);
    nh_.getParam("show_cd", show_cd_);
    nh_.getParam("display_accumulation_time", display_acc_time_);

    const std::string topic_cam_info         = "/prophesee/" + camera_name + "/camera_info";
    const std::string topic_cd_event_buffer  = "/prophesee/" + camera_name + "/cd_events_buffer";

    // Subscribe to camera info topic
    sub_cam_info_ = nh_.subscribe(topic_cam_info, 1, &PropheseeWrapperViewer::cameraInfoCallback, this);

    // Subscribe to CD buffer topic
    if (show_cd_) {
        callback displayerCDCallback = boost::bind(&CDFrameGenerator::add_events, &cd_frame_generator_, _1);
        sub_cd_events_               = nh_.subscribe(topic_cd_event_buffer, 500, displayerCDCallback);
    }
}

PropheseeWrapperViewer::~PropheseeWrapperViewer() {
    if (!initialized_)
        return;

    // Stop the CD frame generator thread
    if (show_cd_)
        cd_frame_generator_.stop();

    // Destroy the windows
    cv::destroyAllWindows();

    nh_.shutdown();
}

bool PropheseeWrapperViewer::isInitialized() {
    return initialized_;
}

void PropheseeWrapperViewer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    if (initialized_)
        return;

    if ((msg->width != 0) && (msg->height != 0))
        init(msg->width, msg->height);
}

bool PropheseeWrapperViewer::init(const unsigned int &sensor_width, const unsigned int &sensor_height) {
    if (show_cd_) {
        // Define the display window for CD events
        create_window(cd_window_name_, sensor_width, sensor_height, 0, 0);
        // Initialize CD frame generator
        cd_frame_generator_.init(sensor_width, sensor_height);
        cd_frame_generator_.set_display_accumulation_time_us(display_acc_time_);
        // Start CD frame generator thread
        cd_frame_generator_.start();
    }

    initialized_ = true;

    return true;
}

void PropheseeWrapperViewer::create_window(const std::string &window_name, const unsigned int &sensor_width,
                                           const unsigned int &sensor_height, const int &shift_x, const int &shift_y) {
    cv::namedWindow(window_name, CV_GUI_EXPANDED);
    cv::resizeWindow(window_name, sensor_width, sensor_height);
    // move needs to be after resize on apple, otherwise the window stacks
    cv::moveWindow(window_name, shift_x, shift_y);
}

void PropheseeWrapperViewer::showData() {
    if (!show_cd_)
        return;

    if (cd_frame_generator_.get_last_ros_timestamp() < ros::Time::now() - ros::Duration(0.5)) {
        cd_frame_generator_.reset();
        initialized_ = false;
    }

    const auto &cd_frame = cd_frame_generator_.get_current_frame();
    if (!cd_frame.empty()) {
        cv::imshow(cd_window_name_, cd_frame);
    }
}

int process_ui_for(const int &delay_ms) {
    auto then = std::chrono::high_resolution_clock::now();
    int key   = cv::waitKey(delay_ms);
    auto now  = std::chrono::high_resolution_clock::now();
    // cv::waitKey will not wait if no window is opened, so we wait for him, if needed
    std::this_thread::sleep_for(std::chrono::milliseconds(
        delay_ms - std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count()));

    return key;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prophesee_ros_viewer");

    PropheseeWrapperViewer wv;

    while (ros::ok() && !wv.isInitialized()) {
        ros::spinOnce();
    }

    while (ros::ok()) {
        ros::spinOnce();

        wv.showData();

        process_ui_for(33);
    }

    ros::shutdown();

    return 0;
}
