#include "realsense.hpp"
#include "librealsense2/hpp/rs_types.hpp"
#include <unistd.h>
#include <iostream>
#include <ctime>

realsense::realsense()
{
    if_depth = true;
    if_color = true;
    if_gyro = true;
    if_accel = true;
    cfg.enable_all_streams();
    pipe_pro = pipe.start(cfg);
    rs2::device dev = pipe_pro.get_device();
    std::cout << std::setiosflags(std::ios::left);
    std::cout << "---------------Realsense Open Success!---------------" << std::endl;
    std::cout << std::setw(20)
              << "Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << std::setw(20)
              << "Serial Number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::vector<rs2::stream_profile> str_pro = pipe_pro.get_streams();
    for (int i = 0; i < str_pro.size(); i++)
    {
        std::cout << "Stream " << i << std::endl;
        std::cout << std::setw(20)
                  << "Stream format: " << str_pro[i].format() << std::endl;
        std::cout << std::setw(20)
                  << "Stream fps: " << str_pro[i].fps() << std::endl;
    }
    std::cout << "-----------------------------------------------------" << std::endl;
}

realsense::realsense(bool enable_depth, bool enable_color,
                     bool enable_gyro, bool enable_accel)
{
    if_depth = enable_depth;
    if_color = enable_color;
    if_gyro = enable_gyro;
    if_accel = enable_accel;
    if (enable_depth)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH);
    }
    if (enable_color)
    {
        cfg.enable_stream(RS2_STREAM_COLOR);
    }
    if (enable_gyro)
    {
        cfg.enable_stream(RS2_STREAM_GYRO);
    }
    if (enable_accel)
    {
        cfg.enable_stream(RS2_STREAM_ACCEL);
    }
    pipe_pro = pipe.start(cfg);
    rs2::device dev = pipe_pro.get_device();
    std::cout << std::setiosflags(std::ios::left);
    std::cout << "---------------Realsense Open Successfull!---------------" << std::endl;
    std::cout << std::setw(20)
              << "Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << std::setw(20)
              << "Serial Number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::vector<rs2::stream_profile> str_pro = pipe_pro.get_streams();
    for (int i = 0; i < str_pro.size(); i++)
    {
        std::cout << "Stream " << i << std::endl;
        std::cout << std::setw(20)
                  << "Stream format: " << str_pro[i].format() << std::endl;
        std::cout << std::setw(20)
                  << "Stream fps: " << str_pro[i].fps() << std::endl;
    }
    std::cout << "---------------------------------------------------------" << std::endl;
}

realsense::~realsense()
{
}

bool realsense::catch_frame(void)
{
    return pipe.poll_for_frames(&frame_set);
}

void realsense::frame_solve(void)
{
    auto now_time = std::clock();
    static clock_t color_pre_time, depth_pre_time, gyro_pre_time, accel_pre_time;
    if (if_color)
    {
        color_frame_solve();
    }
    if (if_depth)
    {
        depth_frame_solve();
    }
    if (if_gyro)
    {
        gyro_frame_solve();
    }
    if (if_accel)
    {
        accel_frame_solve();
    }
    // std::cout << "GYRO: " << return_gyro_frame().x << " "
    //           << return_gyro_frame().y << " " << return_gyro_frame().z << std::endl;
}

bool realsense::color_frame_solve(void)
{
    color_frame = frame_set.get_color_frame();
    if (!color_frame.get_data_size())
    {
        std::cout << "空" << std::endl;
        return false;
    }
    return true;
}

bool realsense::depth_frame_solve(void)
{
    depth_frame = frame_set.get_depth_frame();
    depth_units = depth_frame.get_units();
    if (!depth_frame.get_data_size())
    {
        return false;
    }
    return true;
}

bool realsense::gyro_frame_solve(void)
{
    gyro_frame = frame_set.first_or_default(RS2_STREAM_GYRO);
    if (!gyro_frame.get_data_size())
    {
        return false;
    }
    return true;
}

bool realsense::accel_frame_solve(void)
{
    accel_frame = frame_set.first_or_default(RS2_STREAM_ACCEL);
    if (!accel_frame.get_data_size())
    {
        return false;
    }
    return true;
}

cv::Mat realsense::return_color_frame(std::string color_type)
try
{
    const int w = color_frame.get_width();
    const int h = color_frame.get_height();
    cv::Mat cv_frame = cv::Mat(cv::Size(w, h), CV_8UC3,
                               (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    if (color_type == "BGR")
    {
        ;
    }
    else if (color_type == "RGB")
    {
        cv::cvtColor(cv_frame, cv_frame, cv::COLOR_BGR2RGB);
    }
    else if (color_type == "GRAY")
    {
        cv::cvtColor(cv_frame, cv_frame, cv::COLOR_BGR2GRAY);
    }
    else
    {
        exce = 1;
        throw exce;
    }
    return cv_frame;
}
catch (int exce)
{
    if (exce == 1)
    {
        std::cerr
            << "Error! You give a wrong color type in function get_color_frame()" << std::endl;
    }
    std::unexpected();
}

cv::Mat realsense::return_depth_frame(void)
{
    const int w = depth_frame.get_width();
    const int h = depth_frame.get_height();
    cv::Mat cv_frame = cv::Mat(cv::Size(w, h), CV_16UC1,
                               (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    // for (auto i = 0; i < cv_frame.rows; i++)
    // {
    //     for (auto j = 0; j < cv_frame.cols; j++)
    //     {
    //         cv_frame.at<uint16_t>(i, j) *= 10;
    //     }
    // }
    return cv_frame;
}

rs2_vector realsense::return_gyro_frame(void)
{
    rs2_vector gyro_data = gyro_frame.get_motion_data();
    return gyro_data;
}

rs2_vector realsense::return_accel_frame(void)
{
    rs2_vector accel_data = accel_frame.get_motion_data();
    return accel_data;
}