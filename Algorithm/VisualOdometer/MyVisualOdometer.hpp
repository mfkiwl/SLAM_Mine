#pragma once

#include "MyFeaturePoints.hpp"
#include "MyPositionEstimation.hpp"

class MyVisualOdometer
{
private:
    enum FeatPotType
    {
        ORB = 0,
    } fp_type;

    cv::Mat camera_inside_param;

    MyFeaturePoints feat_pot;
    MyPositionEstimation pos;

public:
    MyVisualOdometer(cv::Mat K, enum FeatPotType type = ORB);
    ~MyVisualOdometer();

    void set_fp_type(FeatPotType type);

    void fp_match(
        cv::Mat pre_frame, cv::Mat now_frame,
        cv::DescriptorMatcher::MatcherType match_type = cv::DescriptorMatcher::BRUTEFORCE_HAMMING,
        double min_dist = 0, double max_dist = 100);

    void pos_estimate(void);

    std::vector<cv::DMatch> return_fp_matches(void);
    std::vector<std::vector<cv::KeyPoint>> return_fp_keypoints(void);
    std::vector<cv::Mat> return_pos_estimation(void);
};
