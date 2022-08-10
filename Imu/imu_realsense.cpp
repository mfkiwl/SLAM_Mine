#include "imu_realsense.hpp"

void rs_imu_init(rs2::config &cfg)
{
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
}