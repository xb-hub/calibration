 //
// Created by 许斌 on 2021/12/26.
//

#include "CameraCalibration/CameraCalibration.h"
using namespace Camera_Calibration;

CameraCalibration::CameraCalibration() :
    intrinsic_matrix({{3000, 0, 640}, {0, 3000, 480}, {0, 0, 1}}),
    external_matrix({{1, 0, 0, -3000}, {0, 1, 0, -1000}, {0, 0, 1, -500}})
{
    for(int x = 0; x < 7; x++)
    {
        for (int y = 0; y < 7; y++)
        {
            cv::Point3d point;
            point.x = (x + 1) * 250;
            point.y = (y + 1) * 250;
            point.z = 1;
            objpoints.push_back(point);
        }
    }
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
