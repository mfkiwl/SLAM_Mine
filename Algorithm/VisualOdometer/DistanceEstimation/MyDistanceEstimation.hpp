#pragma once

#include <vector>
#include <opencv2/core.hpp>

class MyDistanceEstimation
{
private:
    /* data */
public:
    MyDistanceEstimation(/* args */);
    ~MyDistanceEstimation();

    void triangulation(
        std::vector<cv::KeyPoint> keypoint_1,
        std::vector<cv::KeyPoint> keypoint_2,
        std::vector<cv::DMatch> matches,
        cv::Mat K,
        cv::Mat R, cv::Mat t,
        std::vector<cv::Point3d> points);
};
