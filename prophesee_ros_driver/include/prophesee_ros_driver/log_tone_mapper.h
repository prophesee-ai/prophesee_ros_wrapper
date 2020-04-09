/*******************************************************************
 * File : log_tone_mapper.h                                        *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef LOG_TONE_MAPPER_H_
#define LOG_TONE_MAPPER_H_

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

#include <metavision/sdk/driver/prophesee_driver.h>

/// \brief Utility class to display EM events
class LogToneMapper {
public:
    /// \brief Constructor
    ///
    /// @param percent_to_exclude : the percentage of excluded pixels used
    ///        when estimating the black and white point of the image
    /// @param sampling_step_pixel : step used to skip over pixels not considered
    ///        as samples for the LUT estimation for efficency purpose
    LogToneMapper(float percent_to_exclude = 0.015, std::uint16_t sampling_step_pixel = 5);

    cv::Mat operator()(const cv::Mat &exposure_frame);

private:
    void estimate_tone_mapping_parameters(const cv::Mat &exposure_frame, std::int32_t &min_dt, std::int32_t &max_dt);
    void apply_tone_mapping(const cv::Mat &exposure_frame, std::int32_t min_dt, std::int32_t max_dt);

    // Tone mapped image
    cv::Mat frame_;

    // LUT used for log tone mapping
    cv::Mat lut_;

    // Samples used to estimate the LUT parameters
    std::vector<std::int32_t> samples_;
    float excluded_pixels_ratio_;
    int sample_step_;

    // Temp matrices
    cv::Mat tmp_, tmp2_, tmp3_;
};

#endif /* LOG_TONE_MAPPER_H_ */
