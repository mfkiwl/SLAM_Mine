#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

class MyFeaturePoints
{
private:
    enum FPType
    {
        ORB = 0,
    } fp_type;

public:
    MyFeaturePoints();
    MyFeaturePoints(FPType type);
    ~MyFeaturePoints();

    std::vector<cv::DMatch> ORB_Match(
        cv::Mat pre_frame, cv::Mat now_frame,
        cv::DescriptorMatcher::MatcherType match_type = cv::DescriptorMatcher::BRUTEFORCE_HAMMING,
        double min_dist = 0, double max_dist = 100);
};
