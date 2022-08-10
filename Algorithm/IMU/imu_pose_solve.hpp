#pragma once
#include "realsense.hpp"
#include "Fusion.h"
#include <cmath>

class imu_pose
{
private:
    FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};

    FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    FusionOffset offset;
    FusionAhrs ahrs;

    FusionEuler euler;
    FusionVector earth;

public:
    imu_pose(/* args */);
    ~imu_pose();

    bool imu_init(realsense &rs_t);
    void imu_pose_calculate(FusionVector gyroscope, FusionVector accelerometer);

    rs2_vector return_euler(void);
    rs2_vector return_earth(void);

    static inline rs2_vector RS2VectorAddVec(rs2_vector a, rs2_vector b)
    {
        rs2_vector c;
        c.x = a.x + b.x;
        c.y = a.y + b.y;
        c.z = a.z + b.z;
        return c;
    }

    static inline rs2_vector RS2VectorSubVec(rs2_vector a, rs2_vector b)
    {
        rs2_vector c;
        c.x = a.x - b.x;
        c.y = a.y - b.y;
        c.z = a.z - b.z;
        return c;
    }

    static inline rs2_vector RS2VectorDivVec(rs2_vector a, rs2_vector b)
    {
        rs2_vector c;
        c.x = a.x / b.x;
        c.y = a.y / b.y;
        c.z = a.z / b.z;
        return c;
    }

    static inline rs2_vector RS2VectorDivNum(rs2_vector a, float b)
    {
        rs2_vector c;
        c.x = a.x / b;
        c.y = a.y / b;
        c.z = a.z / b;
        return c;
    }

    static inline float RS2VectorModule(rs2_vector a)
    {
        return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    }

    static inline FusionVector RS2VecToFusionVec(rs2_vector a)
    {
        FusionVector b;
        b.axis.x = a.x;
        b.axis.y = a.y;
        b.axis.z = a.z;
        return b;
    }
};
