#pragma once

#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void pose_estimation_2d2d(std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches,
                          cv::Mat &R, cv::Mat &t);

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);