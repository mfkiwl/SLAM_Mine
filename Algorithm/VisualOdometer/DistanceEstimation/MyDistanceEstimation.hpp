#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

class MyDistanceEstimation
{
private:
    std::vector<cv::Point3d> triangle_points;

public:
    MyDistanceEstimation(/* args */);
    ~MyDistanceEstimation();

    void triangulation(
        std::vector<cv::KeyPoint> keypoint_1,
        std::vector<cv::KeyPoint> keypoint_2,
        std::vector<cv::DMatch> matches,
        cv::Mat K,
        cv::Mat R, cv::Mat t);

    inline cv::Point2d pixel2cam(cv::Point2d p, cv::Mat K)
    {
        return cv::Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
    };

    inline cv::Scalar get_color(float depth)
    {
        float up_th = 50, low_th = 10, th_range = up_th - low_th;
        if (depth > up_th)
            depth = up_th;
        if (depth < low_th)
            depth = low_th;
        return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
    }

    std::vector<cv::Point3d> return_triangle_points(void);
};
