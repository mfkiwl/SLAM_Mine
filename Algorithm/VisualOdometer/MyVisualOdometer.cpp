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
    double min_dist, double max_dist,
    cv::DescriptorMatcher::MatcherType match_type)
{
    switch (fp_type)
    {
    case ORB:
        feat_pot.orb_match(pre_frame, now_frame, min_dist, max_dist, match_type);
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

void MyVisualOdometer::dist_estimate(void)
{
    dist.triangulation(feat_pot.return_keypoints()[0], feat_pot.return_keypoints()[1],
                       feat_pot.return_matches(), camera_inside_param,
                       pos.return_estimation()[0], pos.return_estimation()[1]);
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

std::vector<cv::Point3d> MyVisualOdometer::return_dist_estimation(void)
{
    return dist.return_triangle_points();
}