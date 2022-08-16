#include "MyThread.hpp"
#include <iostream>
#include <unistd.h>
#include <chrono>
/*

线程模板

void MyThread::Thread(long fps)
{
    long Thread_RSDataSolve_FPS = fps;
    auto start = std::chrono::high_resolution_clock::now();
    for (;;)
    {
        auto now = std::chrono::high_resolution_clock::now();

        ----code here----

        std::cout << "Total FPS is: " << FPS_Calc(start, now) << std::endl;
        auto timepoint = start + std::chrono::milliseconds(FPS2MillTime(Thread_RSDataSolve_FPS));
        std::this_thread::sleep_until(timepoint);
    }
}

*/

MyThread::MyThread(/* args */)
{
    rs_t = new RealSense();
    // rs_t = new RealSense(true, true, true, true);
    imu_t = new ImuPose();
    vo_t = new MyVisualOdometer(rs_t->return_camera_inside_param());
    Thread_RSDataCatch = new std::thread(std::mem_fn(&MyThread::RSDataCatch), this, 50);
    Thread_RSPoseSolve = new std::thread(std::mem_fn(&MyThread::RSPoseSolve), this, 100);
    Thread_SLAMTest1 = new std::thread(std::mem_fn(&MyThread::SLAMTest1), this);
    Thread_RSDataCatch->join();
    Thread_RSPoseSolve->join();
    Thread_SLAMTest1->join();
}

MyThread::~MyThread()
{
    delete rs_t, imu_t, Thread_RSDataCatch, Thread_RSPoseSolve, Thread_SLAMTest1;
}

void MyThread::RSDataCatch(long fps)
{
    long Thread_RSDataSolve_FPS = fps;
    auto start = std::chrono::high_resolution_clock::now();

    // 开始抓取图像直到有图像产生,防止IMU初始化的时候发散
    while (!rs_t->catch_frame())
    {
        ;
    }
    std::cout << "RealSense Initialized Successfull" << std::endl;
    rs_t->frame_solve();
    realsense_init_flag = true;

    for (;;)
    {
        auto now = std::chrono::high_resolution_clock::now();
        if (rs_t->catch_frame())
        {
            // std::cout << "Inside FPS is: " << FPS_Calc(start, now) << std::endl;
            rs_t->frame_solve();
        }
        // std::cout << "Total FPS is: " << FPS_Calc(start, now) << std::endl;
        auto timepoint = start + std::chrono::milliseconds(FPS2MillTime(Thread_RSDataSolve_FPS));
        std::this_thread::sleep_until(timepoint);
    }
}

void MyThread::RSPoseSolve(long fps)
{
    long Thread_RSDataSolve_FPS = fps;
    auto start = std::chrono::high_resolution_clock::now();

    for (;;)
    {
        if (!realsense_init_flag)
        {
            continue;
        }
        if (!imu_init_flag)
        {
            if (imu_t->imu_init(*rs_t))
            {
                std::cout << "imu_t Initialized Successfull" << std::endl;
                imu_init_flag = true;
            }
            else
            {
                imu_init_flag = false;
            }
        }

        auto now = std::chrono::high_resolution_clock::now();

        imu_t->imu_pose_calculate(imu_t->RS2WToFusionW(imu_t->RS2VecToFusionVec(rs_t->return_gyro_frame())),
                                  imu_t->RS2AToFusionA(imu_t->RS2VecToFusionVec(rs_t->return_accel_frame())));

        // std::cout << "Euler: " << imu_t->return_euler().x << " " << imu_t->return_euler().y
        //           << " " << imu_t->return_euler().z << " " << std::endl;

        // std::cout << "Total FPS is: " << FPS_Calc(start, now) << std::endl;
        auto timepoint = start + std::chrono::milliseconds(FPS2MillTime(Thread_RSDataSolve_FPS));
        std::this_thread::sleep_until(timepoint);
    }
}

void MyThread::SLAMTest1(void)
{
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat pre_frame, now_frame, img_frame;
    for (;;)
    {
        if (!realsense_init_flag || !imu_init_flag)
        {
            continue;
        }
        auto now = std::chrono::high_resolution_clock::now();
        // std::cout << "SLAM FPS is: " << FPS_Calc(start, now) << std::endl;

        pre_frame = now_frame;
        now_frame = rs_t->return_color_frame("RGB");
        if (pre_frame.empty() || now_frame.empty())
        {
            continue;
        }
        vo_t->fp_match(pre_frame, now_frame);
        cv::drawMatches(pre_frame, vo_t->return_fp_keypoints()[0],
                        now_frame, vo_t->return_fp_keypoints()[1],
                        vo_t->return_fp_matches(), img_frame);
        vo_t->pos_estimate();
        std::cout << "t:" << vo_t->return_pos_estimation()[1] << "\r" << std::endl;
        cv::imshow("Matches", img_frame);
        cv::waitKey(1);
    }
}