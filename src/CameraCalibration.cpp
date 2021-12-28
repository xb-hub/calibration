//
// Created by 许斌 on 2021/12/26.
//

#include "CameraCalibration/CameraCalibration.h"
using namespace Camera_Calibration;

CameraCalibration::CameraCalibration()
{
    Eigen::Matrix3d circle1;
    circle1 << 1, 0, 0,
              0, 1, 0,
              0, 0, -1;
    concenteric_circle.push_back(circle1);
    Eigen::Matrix3d circle2;
    circle2 << 1, 0, 0,
               0, 1, 0,
               0, 0, -4;
    concenteric_circle.push_back(circle2);
    Eigen::Matrix3d circle3;
    circle3 << 1, 0, 0,
               0, 1, 0,
               0, 0, -9;
    concenteric_circle.push_back(circle3);
    Eigen::Matrix3d circle4;
    circle4 << 1, 0, 0,
               0, 1, 0,
               0, 0, -16;
    concenteric_circle.push_back(circle4);
    Eigen::Vector3d pole1;
    pole1 << 6,
             0,
             1;
    poles.push_back(pole1);
    Eigen::Vector3d pole2;
    pole2 << 8,
             8,
             1;
    poles.push_back(pole2);
    intrinsic_matrix << 50, 0, 150, 0,
                        0, 50, 150, 0,
                        0, 0, 1, 0;
}

CameraCalibration::~CameraCalibration()
{

}

cv::Point2d CameraCalibration::getLineIntersection(Eigen::Vector3d l1, Eigen::Vector3d l2)
{
    Eigen::Vector3d temp = l1.cross(l2);
    cv::Point2d pole;
    Eigen2Point(temp, pole);
    return pole;
}

Eigen::Vector3d CameraCalibration::getPolar(Eigen::Matrix3d conic, Eigen::Vector3d pole)
{
    return conic * pole;
}

cv::Point2d CameraCalibration::getPole(Eigen::Matrix3d conic, Eigen::Vector3d polar)
{
    Eigen::Vector3d temp = conic * polar;
    cv::Point2d pole;
    Eigen2Point(temp, pole);
    return pole;
}

void CameraCalibration::Eigen2Point(Eigen::Vector3d pole, cv::Point2d& point)
{
    point.x = pole(0, 0) / pole(2, 0);
    point.y = pole(1, 0) / pole(2, 0);
}
