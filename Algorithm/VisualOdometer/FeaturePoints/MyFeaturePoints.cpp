#include "MyFeaturePoints.hpp"

MyFeaturePoints::MyFeaturePoints()
{
    fp_type = ORB;
}

MyFeaturePoints::MyFeaturePoints(FPType type)
{
    fp_type = type;
}

MyFeaturePoints::~MyFeaturePoints()
{
}

std::vector<cv::DMatch> MyFeaturePoints::ORB_Match(
    cv::Mat pre_frame, cv::Mat now_frame,
    cv::DescriptorMatcher::MatcherType match_type,
    double min_dist, double max_dist)
{
    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::KeyPoint> pre_keypoints, now_keypoints;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(match_type);

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(pre_frame, pre_keypoints);
    detector->detect(now_frame, now_keypoints);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(pre_frame, pre_keypoints, descriptors_1);
    descriptor->compute(now_frame, now_keypoints, descriptors_2);

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

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= std::max(2 * min_dist, 20.0))
        {
            good_match.push_back(match[i]);
        }
    }

    return good_match;
}