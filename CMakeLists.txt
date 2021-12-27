cmake_minimum_required(VERSION 3.20)
project(calibration)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(calibration main.cpp src/CameraCalibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${Eigen3_LIBRARIES})