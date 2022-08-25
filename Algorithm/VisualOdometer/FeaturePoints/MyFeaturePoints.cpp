#include "MyFeaturePoints.hpp"
#include <iostream>

MyFeaturePoints::MyFeaturePoints()
{
}

MyFeaturePoints::~MyFeaturePoints()
{
}

void MyFeaturePoints::orb_match(
    cv::Mat pre_frame, cv::Mat now_frame,
    double min_dist, double max_dist,
    cv::DescriptorMatcher::MatcherType match_type)
{
    //-- 初始化
    cv::Mat frame_1, frame_2;
    frame_1 = pre_frame;
    frame_2 = now_frame;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(match_type);

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(frame_1, keypoints_1);
    detector->detect(frame_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(frame_1, keypoints_1, descriptors_1);
    descriptor->compute(frame_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，默认使用 Hamming 距离
    std::vector<cv::DMatch> match, good_match;
    matcher->match(descriptors_1, descriptors_2, match);

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    std::cout << "min_dist: " << min_dist << std::endl;

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= std::max(1.5 * min_dist, 10.0))
        {
            good_match.push_back(match[i]);
        }
    }

    matches = good_match;
    pre_keypoints = keypoints_1;
    now_keypoints = keypoints_2;
}

std::vector<cv::DMatch> MyFeaturePoints::return_matches(void)
{
    return matches;
}

std::vector<std::vector<cv::KeyPoint>> MyFeaturePoints::return_keypoints(void)
{
    std::vector<std::vector<cv::KeyPoint>> a;
    a.push_back(pre_keypoints);
    a.push_back(now_keypoints);
    return a;
}