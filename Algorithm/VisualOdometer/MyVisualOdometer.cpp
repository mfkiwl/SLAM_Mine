#include "MyVisualOdometer.hpp"
#include <iostream>

MyVisualOdometer::MyVisualOdometer(cv::Mat K, enum FeatPotType type)
{
    camera_inside_param = K;
    fp_type = type;
}

MyVisualOdometer::~MyVisualOdometer()
{
}

void MyVisualOdometer::set_fp_type(FeatPotType type)
{
    fp_type = type;
}

void MyVisualOdometer::fp_match(
    cv::Mat pre_frame, cv::Mat now_frame,
    cv::DescriptorMatcher::MatcherType match_type,
    double min_dist, double max_dist)
{
    switch (fp_type)
    {
    case ORB:
        feat_pot.orb_match(pre_frame, now_frame, match_type, min_dist, max_dist);
        break;

    default:
        break;
    }
}

void MyVisualOdometer::pos_estimate(void)
{
    pos.pos_estimate_2d2d(feat_pot.return_keypoints()[0], feat_pot.return_keypoints()[1],
                          feat_pot.return_matches(), camera_inside_param);
}

std::vector<cv::DMatch> MyVisualOdometer::return_fp_matches(void)
{
    return feat_pot.return_matches();
}

std::vector<std::vector<cv::KeyPoint>> MyVisualOdometer::return_fp_keypoints(void)
{
    return feat_pot.return_keypoints();
}

std::vector<cv::Mat> MyVisualOdometer::return_pos_estimation(void)
{
    return pos.return_estimation();
}