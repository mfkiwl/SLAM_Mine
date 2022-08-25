#include "MyPositionEstimation.hpp"
#include <iostream>

MyPositionEstimation::MyPositionEstimation(/* args */)
{
}

MyPositionEstimation::~MyPositionEstimation()
{
}

void MyPositionEstimation::pos_estimate_2d2d(
    std::vector<cv::KeyPoint> keypoints_1,
    std::vector<cv::KeyPoint> keypoints_2,
    std::vector<cv::DMatch> matches,
    cv::Mat K)
{
    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    if ((points1.size() >= 5) && (points2.size() >= 5))
    {
        //-- 计算本质矩阵
        cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, K);

        std::vector<cv::Mat> a;
        cv::Mat R, t;

        //-- 从本质矩阵中恢复旋转和平移信息.
        if (essential_matrix.cols == 3 && essential_matrix.rows == 3)
        {
            cv::recoverPose(essential_matrix, points1, points2, K, R, t);
            a.push_back(R);
            a.push_back(t);
            estimation_matrix = a;
        }
    }
    else
    {
        std::cout << "points < 5 !!! Not Enough information!!!!" << std::endl;
    }
}

void MyPositionEstimation::pos_estimate_3d2d(void)
{
}

std::vector<cv::Mat> MyPositionEstimation::return_estimation(void)
{
    return estimation_matrix;
}
