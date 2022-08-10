#include "pose_estimation.hpp"
#include <iostream>

void pose_estimation_2d2d(std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches,
                          cv::Mat &R, cv::Mat &t)
{
    // 相机内参,TUM Freiburg2
    // cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 相机内参,Intel Realsense D435i
    cv::Mat K = (cv::Mat_<double>(3, 3) << 605.926, 0, 317.4, 0, 605.7, 246.356, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    // std::cout << "fundamental_matrix is " << std::endl
    //           << fundamental_matrix << std::endl;

    //-- 计算本质矩阵
    cv::Point2d principal_point(317.4, 246.356); //相机光心
    double focal_length = 605.8;                 //相机焦距
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    // std::cout << "essential_matrix is " << std::endl
    //           << essential_matrix << std::endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
    // std::cout << "homography_matrix is " << std::endl
    //           << homography_matrix << std::endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 此函数仅在Opencv3中提供
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    std::cout << "R is " << std::endl
              << R << std::endl;
    // std::cout << "t is " << std::endl
    //           << t << "\r";
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
    return cv::Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}