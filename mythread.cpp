#include "mythread.hpp"
#include <iostream>
#include <unistd.h>
#include <chrono>
/*

线程模板

void mythread::Thread(long fps)
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

mythread::mythread(/* args */)
{
    rs_t = new realsense();
    // rs_t = new realsense(true, true, true, true);
    imu = new imu_pose();
    Thread_RSDataCatch = new std::thread(std::mem_fn(&mythread::RSDataCatch), this, 50);
    Thread_RSPoseSolve = new std::thread(std::mem_fn(&mythread::RSPoseSolve), this, 100);
    Thread_RSDataCatch->join();
    Thread_RSPoseSolve->join();
}

mythread::~mythread()
{
}

void mythread::RSDataCatch(long fps)
{
    long Thread_RSDataSolve_FPS = fps;
    auto start = std::chrono::high_resolution_clock::now();

    // 开始抓取图像直到有图像产生,防止IMU初始化的时候发散
    while (!rs_t->catch_frame())
    {
        ;
    }
    std::cout << "Realsense Initialized Successfull" << std::endl;
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

void mythread::RSPoseSolve(long fps)
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
            if (imu->imu_init(*rs_t))
            {
                std::cout << "IMU Initialized Successfull" << std::endl;
                imu_init_flag = true;
            }
            else
            {
                imu_init_flag = false;
            }
        }

        auto now = std::chrono::high_resolution_clock::now();

        imu->imu_pose_calculate(imu->RS2WToFusionW(imu->RS2VecToFusionVec(rs_t->return_gyro_frame())),
                                imu->RS2AToFusionA(imu->RS2VecToFusionVec(rs_t->return_accel_frame())));

        std::cout << "Euler: " << imu->return_euler().x << " " << imu->return_euler().y
                  << " " << imu->return_euler().z << " " << std::endl;

        // std::cout << "Total FPS is: " << FPS_Calc(start, now) << std::endl;
        auto timepoint = start + std::chrono::milliseconds(FPS2MillTime(Thread_RSDataSolve_FPS));
        std::this_thread::sleep_until(timepoint);
    }
}