#pragma once
#include "realsense.hpp"
#include <cmath>

bool imu_init(realsense &rs_t);

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