#include "MyDistanceEstimation.hpp"

MyDistanceEstimation::MyDistanceEstimation(/* args */)
{
}

MyDistanceEstimation::~MyDistanceEstimation()
{
}

void MyDistanceEstimation::triangulation(
    std::vector<cv::KeyPoint> keypoint_1,
    std::vector<cv::KeyPoint> keypoint_2,
    std::vector<cv::DMatch> matches,
    cv::Mat K,
    cv::Mat R, cv::Mat t)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                  R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                  R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    std::vector<cv::Point2f> pts_1, pts_2;
    for (cv::DMatch m : matches)
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    std::vector<cv::Point3d> points;

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        points.push_back(p);
    }

    triangle_points = points;
}

std::vector<cv::Point3d> MyDistanceEstimation::return_triangle_points(void)
{
    return triangle_points;
}