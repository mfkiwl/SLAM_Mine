#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class MyPositionEstimation
{
private:
    std::vector<cv::Mat> estimation_matrix;

public:
    MyPositionEstimation(/* args */);
    ~MyPositionEstimation();

    void pos_estimate_2d2d(
        std::vector<cv::KeyPoint> keypoints_1,
        std::vector<cv::KeyPoint> keypoints_2,
        std::vector<cv::DMatch> matches,
        cv::Mat K);

    void pos_estimate_3d2d(void);

    std::vector<cv::Mat> return_estimation(void);
};
