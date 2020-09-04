/*******************************************************************
 * File : log_tone_mapper.cpp                                      *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include "log_tone_mapper.h"

LogToneMapper::LogToneMapper(float percent_to_exclude, std::uint16_t sampling_step_pixel) :
    lut_(1, 256, CV_8U),
    excluded_pixels_ratio_(percent_to_exclude),
    sample_step_(sampling_step_pixel) {}

void LogToneMapper::estimate_tone_mapping_parameters(const cv::Mat &exposure_frame, std::int32_t &min_dt,
                                                     std::int32_t &max_dt) {
    const int width          = exposure_frame.cols;
    const int height         = exposure_frame.rows;
    const size_t num_samples = static_cast<size_t>(width / sample_step_ * height / sample_step_);
    samples_.resize(num_samples);
    int count = 0;
    for (int j = 0; j < height; j += sample_step_) {
        const std::int32_t *ptr = exposure_frame.ptr<std::int32_t>(j);
        for (int i = 0; i < width; i += sample_step_) {
            if (ptr[i] < std::numeric_limits<std::int32_t>::max()) {
                samples_[count++] = ptr[i];
            }
        }
    }
    if (count < 1)
        return;

    const size_t min_pos = static_cast<size_t>(count * excluded_pixels_ratio_);
    const size_t max_pos = count - 1 - min_pos;
    std::nth_element(samples_.begin(), samples_.begin() + min_pos, samples_.begin() + count);
    std::nth_element(samples_.begin(), samples_.begin() + max_pos, samples_.begin() + count);
    min_dt = std::max(samples_[min_pos], std::int32_t(1));
    max_dt = std::min(samples_[max_pos], std::int32_t(1E6));
}

void LogToneMapper::apply_tone_mapping(const cv::Mat &exposure_frame, std::int32_t min_dt, std::int32_t max_dt) {
    if (min_dt < max_dt) {
        // normalize the inverse values to 8 bit precision
        cv::min(exposure_frame, max_dt, tmp_);
        cv::max(tmp_, min_dt, tmp_);
        cv::divide(1.0, tmp_, tmp3_, CV_32F);
        cv::normalize(tmp3_, tmp2_, 0, 255, cv::NORM_MINMAX, CV_8U);

        // build a LUT for efficient log tone mapping
        const float alpha     = 1.0f / (std::log(min_dt) - std::log(max_dt));
        const float beta      = -std::log(max_dt) * alpha;
        const float step      = (1.0f / min_dt - 1.0f / max_dt) / 255.f;
        std::uint8_t *lut_ptr = lut_.ptr<std::uint8_t>();
        lut_ptr[0]            = 0;
        lut_ptr[255]          = 255;
        for (int i = 1; i < 255; ++i)
            lut_ptr[i] = 255 * (alpha * -std::log(1.0f / max_dt + i * step) + beta);

        // apply the LUT
        cv::LUT(tmp2_, lut_, frame_);
    }
}

cv::Mat LogToneMapper::operator()(const cv::Mat &exposure_frame) {
    std::int32_t min_dt = 0, max_dt = 0;
    estimate_tone_mapping_parameters(exposure_frame, min_dt, max_dt);
    apply_tone_mapping(exposure_frame, min_dt, max_dt);
    return frame_;
}
