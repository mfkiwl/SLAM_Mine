cmake_minimum_required(VERSION 3.0.0)
project(slam VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    Camera/RealSense

    Algorithm
    Algorithm/IMU
    Algorithm/IMU/Fusion

    Algorithm/VisualOdometer
    Algorithm/VisualOdometer/FeaturePoints

    ${OpenCV_INCLUDE_DIRS}
)

add_executable(slam 
main.cpp 
MyThread.cpp

Camera/RealSense/RealSense.cpp

Algorithm/IMU/ImuPoseSolve.cpp
Algorithm/IMU/Fusion/FusionAhrs.c
Algorithm/IMU/Fusion/FusionCompass.c
Algorithm/IMU/Fusion/FusionOffset.c

Algorithm/VisualOdometer/MyVisualOdometer.cpp
Algorithm/VisualOdometer/FeaturePoints/MyFeaturePoints.cpp

Algorithm/pose_estimation.cpp
Algorithm/distance_estimation.cpp

)

target_link_libraries(
    slam
    realsense2
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBRARIES}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
