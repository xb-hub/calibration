 //
// Created by 许斌 on 2021/12/26.
//

#include "CameraCalibration/CameraCalibration.h"
using namespace Camera_Calibration;

CameraCalibration::CameraCalibration() :
    intrinsic_matrix({{3000, 0, 640}, {0, 3000, 480}, {0, 0, 1}}),
    external_matrix({{1, 0, 0, -3000}, {0, 1, 0, -1000}, {0, 0, 1, -500}})
{
    Eigen::Matrix3d circle1;
    circle1 << 1, 0, 0,
              0, 1, 0,
              0, 0, -90000;
    concenteric_circle.push_back(circle1);
    Eigen::Matrix3d circle2;
    circle2 << 1, 0, 0,
               0, 1, 0,
               0, 0, -160000;
    concenteric_circle.push_back(circle2);
    Eigen::Matrix3d circle3;
    circle3 << 1, 0, 0,
               0, 1, 0,
               0, 0, -250000;
    concenteric_circle.push_back(circle3);
    Eigen::Matrix3d circle4;
    circle4 << 1, 0, 0,
               0, 1, 0,
               0, 0, -360000;
    concenteric_circle.push_back(circle4);
    Eigen::Vector3d pole1;
    pole1 << 700,
             0,
             1;
    poles.push_back(pole1);
    Eigen::Vector3d pole2;
    pole2 << 800,
             800,
             1;
    poles.push_back(pole2);
    H_matrix.col(0) = (intrinsic_matrix * external_matrix).col(0);
    H_matrix.col(1) = (intrinsic_matrix * external_matrix).col(1);
    H_matrix.col(2) = (intrinsic_matrix * external_matrix).col(3);
    std::cout << "H_matrix:" << std::endl << H_matrix << std::endl;
}

CameraCalibration::~CameraCalibration()
{

}

cv::Point2d CameraCalibration::getLineIntersection(Eigen::Vector3d l1, Eigen::Vector3d l2)
{
    Eigen::Vector3d temp = l1.cross(l2);
    cv::Point2d point;
    Eigen2Point(temp, point);
    return point;
}

Eigen::Vector3d CameraCalibration::getPolar(Eigen::Matrix3d conic, Eigen::Vector3d pole)
{
    return conic * pole;
}

cv::Point2d CameraCalibration::getPole(Eigen::Matrix3d conic, Eigen::Vector3d polar)
{
    Eigen::Vector3d temp = conic.inverse() * polar;
    cv::Point2d pole;
    Eigen2Point(temp, pole);
    return pole;
}



Eigen::Matrix3d CameraCalibration::getProjectConic(const Eigen::Matrix3d& conic) const
{
    return H_matrix.inverse().transpose() * conic * H_matrix.inverse();
}

Eigen::Vector3d CameraCalibration::getProjectPoint(const Eigen::Vector3d& point) const
{
    return H_matrix * point;
}

Eigen::Vector3d CameraCalibration::getLineMatr(std::vector<cv::Point2d> line)
{
    cv::Vec4f line_para;
    cv::fitLine(line, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

    double k_line = line_para[1] / line_para[0];
    double a = k_line;
    double b = -1;
    double c =  line_para[3] - k_line * line_para[2];
    Eigen::Vector3d l;
    l <<  a,
          b,
          c;
    return l;
}

void CameraCalibration::Eigen2Point(const Eigen::Vector3d& pole, cv::Point2d& point)
{
    point.x = pole(0, 0) / pole(2, 0);
    point.y = pole(1, 0) / pole(2, 0);
}
