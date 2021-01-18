/*******************************************************************
 * File : cd_frame_generator.h                                     *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef CD_FRAME_GENERATOR_H_
#define CD_FRAME_GENERATOR_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>

/// \brief Utility class to display CD events
class CDFrameGenerator {
public:
    /// \brief Default constructor
    CDFrameGenerator();

    /// \brief Destructor
    ~CDFrameGenerator();

    /// \brief Initializes the frame generator
    ///
    /// @param width, height : size of the image
    void init(long width, long height);

    /// \brief Adds the buffer of events to be displayed
    ///
    /// @param msg : event buffer message
    void add_events(const prophesee_event_msgs::EventArray::ConstPtr &msg);

    /// \brief Sets the time interval to display events
    ///
    /// The events shown at each refresh are such that their timestamps are in the last 'display_accumulation_time_us'
    /// microseconds from the last received event timestamp.
    /// \param display_accumulation_time_us the time interval to display events from up to now, in us.
    void set_display_accumulation_time_us(long long display_accumulation_time_us);

    /// \brief Starts the generator thread
    ///
    /// After this, you can call get_current_frame() whenever you want to have the
    /// most up to date CD frame to display
    /// \return True if the thread started successfully, false otherwise.
    /// Also returns false, if the thread is already started.
    bool start();

    /// \brief Stops the generator thread
    ///
    /// \return True if the thread has been stopped successfully, false otherwise.
    //  If the thread is not started, this function returns false.
    bool stop();

    /// \brief Resets the frame, the queue of events, and the last processed timestamp
    void reset();

    /// \brief Gets the current most up to date CD frame
    ///
    /// \return the current frame
    const cv::Mat &get_current_frame();

    /// \brief Gets the last event timestamp in us
    ///
    /// \return the last event timestamp
    const Metavision::timestamp &get_last_event_timestamp() const;

    /// \brief Gets ROS time of the last received event buffer
    ///
    /// \return ROS time of the last received event buffer
    const ros::Time &get_last_ros_timestamp() const;

    /// \brief Converts ROS time into Metavision timestamp
    ///
    /// \return Timestamp in microseconds
    inline Metavision::timestamp ros_timestamp_in_us(const ros::Time &ts) const {
        return static_cast<Metavision::timestamp>(ts.toNSec() / 1000.00);
    };

private:
    // Generate a frame
    void generate();

    // Number of pixels in an image
    size_t pix_count_ = 0;

    // Vector of timestamps in us
    std::vector<Metavision::timestamp> ts_history_;

    // Generated image
    cv::Mat frame_;

    // Image to display
    cv::Mat frame_to_show_;

    // Time interval to display events
    uint32_t display_accumulation_time_us_ = 5000;

    // Last event timestamp in us
    Metavision::timestamp last_ts_ = 0, last_process_ts_ = 0;

    size_t min_events_to_process_                     = 10000;
    Metavision::timestamp max_delay_before_processing_ = 5000;

    // Received events
    std::vector<prophesee_event_msgs::Event> events_queue_front_;

    // Events to display
    std::vector<prophesee_event_msgs::Event> events_queue_back_;

    std::mutex frame_show_mutex_;
    std::mutex processing_mutex_;
    std::atomic<bool> thread_should_process_;

    // The worker thread
    std::thread thread_;
    std::mutex thread_cond_mutex_;
    std::condition_variable thread_cond_;
    std::atomic<bool> thread_should_stop_;

    // Image width
    int width_ = 0;

    // Image height
    int height_ = 0;

    /// \brief State of the frame generator: initialized or not
    bool initialized_ = false;

    // ROS time of the last received event buffer
    ros::Time last_ros_ts_;
};

#endif /* CD_FRAME_GENERATOR_H_ */
