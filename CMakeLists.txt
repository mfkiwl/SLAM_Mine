cmake_minimum_required(VERSION 3.0.0)
project(slam VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    Camera/Realsense
    Algorithm
    Algorithm/Fusion
    Imu
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(slam 
main.cpp 
mythread.cpp
Camera/Realsense/realsense.cpp
Algorithm/feature_match.cpp
Algorithm/pose_estimation.cpp
Algorithm/distance_estimation.cpp
Algorithm/imu_pose_solve.cpp
Algorithm/Fusion/FusionAhrs.c
Algorithm/Fusion/FusionCompass.c
Algorithm/Fusion/FusionOffset.c
Imu/imu_realsense.cpp
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
