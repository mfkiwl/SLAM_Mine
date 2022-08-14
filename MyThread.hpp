#pragma once

#include <vector>
#include <thread>
#include "RealSense.hpp"
#include "ImuPoseSolve.hpp"
#include "MyVisualOdometer.hpp"

class MyThread
{
private:
    std::thread *Thread_RSDataCatch;
    std::thread *Thread_RSPoseSolve;
    std::thread *Thread_SLAMTest1;
    RealSense *rs_t;
    ImuPose *imu_t;
    MyVisualOdometer *vo_t;
    bool realsense_init_flag = false;
    bool imu_init_flag = false;

public:
    MyThread();
    ~MyThread();
    void RSDataCatch(long fps);
    void RSPoseSolve(long fps);
    void SLAMTest1(void);

    inline float FPS_Calc(
        std::chrono::_V2::system_clock::time_point &pre_time,
        std::chrono::_V2::system_clock::time_point &now_time)
    {
        auto time_s = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - pre_time);
        if (time_s.count())
        {
            float fps = 1000.0f / (float)time_s.count();
            pre_time = now_time;
            return fps;
        }
        else
        {
            return -1;
        }
    }

    inline long FPS2MillTime(long fps)
    {
        return 1000 / fps;
    }
};
