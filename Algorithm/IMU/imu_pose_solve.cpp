#include "imu_pose_solve.hpp"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <thread>

#define SAMPLE_RATE (100)

imu_pose::imu_pose(/* args */)
{
}

imu_pose::~imu_pose()
{
}

bool imu_pose::imu_init(realsense &rs_t)
{
    rs2_vector gyro_total, gyro_ava, gyro_last_ava;
    rs2_vector accel_total, accel_ava, accel_last_ava;
    gyro_total = {0.0f, 0.0f, 0.0f};
    gyro_ava = {0.0f, 0.0f, 0.0f};
    gyro_last_ava = {0.0f, 0.0f, 0.0f};
    accel_total = {0.0f, 0.0f, 0.0f};
    accel_ava = {0.0f, 0.0f, 0.0f};
    accel_last_ava = {0.0f, 0.0f, 0.0f};

    long count = 0;
    long enable_gyro_flag = 0, enable_accel_flag = 0;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    while (true)
    {
        count++;

        gyro_total = RS2VectorAddVec(gyro_total, rs_t.return_gyro_frame());
        gyro_last_ava = gyro_ava;
        gyro_ava = RS2VectorDivNum(gyro_total, (float)count);

        accel_total = RS2VectorAddVec(accel_total, rs_t.return_accel_frame());
        accel_last_ava = accel_ava;
        accel_ava = RS2VectorDivNum(accel_total, (float)count);

        if (RS2VectorModule(RS2VectorSubVec(gyro_ava, gyro_last_ava)) < 0.005f)
        {
            enable_gyro_flag++;
        }
        if (RS2VectorModule(RS2VectorSubVec(accel_ava, accel_last_ava)) < 0.005f)
        {
            enable_accel_flag++;
        }

        // std::cout << "GRYO FLAG: " << enable_gyro_flag << std::endl;
        // std::cout << "ACCEL FLAG: " << enable_accel_flag << std::endl;
        // std::cout << "accel calibration: " << rs_t.return_accel_frame().x - accel_ava.x << " "
        //           << rs_t.return_accel_frame().y - accel_ava.y << " " << rs_t.return_accel_frame().z - accel_ava.z << std::endl;

        if ((enable_gyro_flag > 40) && (enable_accel_flag > 40))
        {
            std::cout << "gyro bias: " << gyro_ava.x << " " << gyro_ava.y << " " << gyro_ava.z << std::endl;
            std::cout << "accel bias: " << accel_ava.x << " " << accel_ava.y << " " << accel_ava.z << std::endl;
            gyroscopeOffset = RS2VecToFusionVec(gyro_ava);
            return true;
        }

        if (count > 100)
        {
            std::cout << "Failed to initialize the IMU!!!!!!!" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

void imu_pose::imu_pose_calculate(FusionVector gyroscope, FusionVector accelerometer)
{
    const FusionVector magnetometer = {0.0f, 0.0f, 0.0f};
    const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
                                       // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static clock_t previousTimestamp;
    const float deltaTime = (float)(timestamp - previousTimestamp) / (float)CLOCKS_PER_SEC;
    previousTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    // Print algorithm outputs
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    earth = FusionAhrsGetEarthAcceleration(&ahrs);

    // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
    //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
    //        earth.axis.x, earth.axis.y, earth.axis.z);
}

rs2_vector imu_pose::return_euler(void)
{
    rs2_vector a;
    a.x = euler.angle.pitch;
    a.y = euler.angle.roll;
    a.z = euler.angle.yaw;
    return a;
}

rs2_vector imu_pose::return_earth(void)
{
    rs2_vector a;
    a.x = earth.axis.x;
    a.y = earth.axis.y;
    a.z = earth.axis.z;
    return a;
}