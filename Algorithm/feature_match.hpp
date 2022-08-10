#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

void ORB_match(const cv::Mat &img_1, const cv::Mat &img_2,
               std::vector<cv::KeyPoint> &keypoints_1,
               std::vector<cv::KeyPoint> &keypoints_2,
               std::vector<cv::DMatch> &matches);