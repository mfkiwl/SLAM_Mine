#pragma once

#include <vector>
#include <thread>
#include "realsense.hpp"
#include "imu_pose_solve.hpp"

class mythread
{
private:
    std::thread *Thread_RSDataCatch;
    std::thread *Thread_RSPoseSolve;
    realsense *rs_t;
    imu_pose *imu;
    bool realsense_init_flag = false;
    bool imu_init_flag = false;

public:
    mythread();
    ~mythread();
    void RSDataCatch(long fps);
    void RSPoseSolve(long fps);

    static inline float FPS_Calc(
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
    static inline long FPS2MillTime(long fps)
    {
        return 1000 / fps;
    }
};
