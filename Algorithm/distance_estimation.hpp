#pragma once
#include <opencv2/core.hpp>

void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R, const cv::Mat &t,
    std::vector<cv::Point3d> &points);