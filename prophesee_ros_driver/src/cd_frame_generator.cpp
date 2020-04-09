/*******************************************************************
 * File : cd_frame_generator.cpp                                   *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include "cd_frame_generator.h"

CDFrameGenerator::CDFrameGenerator() {}

CDFrameGenerator::~CDFrameGenerator() {
    stop();
}

void CDFrameGenerator::init(long width, long height) {
    width_     = width;
    height_    = height;
    pix_count_ = height_ * width_;
    ts_history_.resize(pix_count_);

    frame_         = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(128));
    frame_to_show_ = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(128));

    initialized_ = true;
}

void CDFrameGenerator::add_events(const prophesee_event_msgs::EventArray::ConstPtr &msgs) {
    bool should_process = false;
    {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        if (std::begin(msgs->events) < std::end(msgs->events)) {
            events_queue_front_.insert(events_queue_front_.end(), std::begin(msgs->events), std::end(msgs->events));
            last_ts_ = ros_timestamp_in_us((std::end(msgs->events) - 1)->ts);
        }
        if (events_queue_front_.size() >= min_events_to_process_ ||
            last_ts_ >= last_process_ts_ + max_delay_before_processing_) {
            should_process = true;
        }
    }

    if (should_process) {
        { thread_should_process_ = true; }
        thread_cond_.notify_one();
    }

    last_ros_ts_ = ros::Time::now();
}

void CDFrameGenerator::reset() {
    if (!initialized_)
        return;

    {
        std::unique_lock<std::mutex> lock_process(processing_mutex_);
        thread_should_process_ = false;
        last_ts_               = 0;
        last_process_ts_       = 0;

        // Clean the queues with events
        events_queue_front_.clear();
        events_queue_back_.clear();
    }

    {
        // Clean the frame
        std::unique_lock<std::mutex> lock_show(frame_show_mutex_);
        frame_.setTo(cv::Scalar(128));
        frame_to_show_.setTo(cv::Scalar(128));

        // Clean the pixels timstamps history
        ts_history_.assign(ts_history_.size(), 0);
    }

    initialized_ = false;
}

void CDFrameGenerator::generate() {
    if (!initialized_)
        return;

    while (true) {
        {
            std::unique_lock<std::mutex> lock(thread_cond_mutex_);
            thread_cond_.wait(lock, [this] { return thread_should_process_ || thread_should_stop_; });
        }

        if (thread_should_stop_)
            break;

        Metavision::timestamp threshold_to_display;
        {
            std::unique_lock<std::mutex> lock(processing_mutex_);
            std::swap(events_queue_front_, events_queue_back_);
            if (!events_queue_back_.empty())
                last_process_ts_ = ros_timestamp_in_us(events_queue_back_.back().ts);
            threshold_to_display = last_ts_ - display_accumulation_time_us_;
        }

        if (events_queue_back_.empty())
            continue;

        std::unique_lock<std::mutex> lock(frame_show_mutex_);
        for (auto it = events_queue_back_.rbegin(), it_end = events_queue_back_.rend();
             it != it_end && ros_timestamp_in_us(it->ts) >= threshold_to_display; ++it) {
            Metavision::timestamp &p_ts = ts_history_[it->y * width_ + it->x];
            if (p_ts < ros_timestamp_in_us(it->ts)) {
                p_ts                             = ros_timestamp_in_us(it->ts);
                frame_.at<uint8_t>(it->y, it->x) = it->polarity ? 255 : 0;
            }
        }

        auto ts_history_it = ts_history_.begin();
        auto datas         = frame_.data;
        for (size_t i = 0; i < pix_count_; ++i, ++ts_history_it, ++datas) {
            if (*ts_history_it < threshold_to_display) {
                *datas = 128;
            }
        }

        events_queue_back_.clear();
        thread_should_process_ = false;
    }
}

void CDFrameGenerator::set_display_accumulation_time_us(Metavision::timestamp display_accumulation_time_us) {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    display_accumulation_time_us_ = display_accumulation_time_us;
}

const cv::Mat &CDFrameGenerator::get_current_frame() {
    std::unique_lock<std::mutex> lock(frame_show_mutex_);
    std::swap(frame_to_show_, frame_);
    return frame_to_show_;
}

const Metavision::timestamp &CDFrameGenerator::get_last_event_timestamp() const {
    return last_ts_;
}

const ros::Time &CDFrameGenerator::get_last_ros_timestamp() const {
    return last_ros_ts_;
}

bool CDFrameGenerator::start() {
    if (!initialized_)
        return false;

    if (thread_.joinable()) { // Already started
        return false;
    }

    std::unique_lock<std::mutex> lock(thread_cond_mutex_);
    thread_should_process_ = false;
    thread_should_stop_    = false;
    thread_                = std::thread([this] { generate(); });

    return true;
}

bool CDFrameGenerator::stop() {
    if (!thread_.joinable()) {
        return false;
    }

    { thread_should_stop_ = true; }

    thread_cond_.notify_one();
    thread_.join();
    return true;
}
