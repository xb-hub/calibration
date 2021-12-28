#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include "CameraCalibration/CameraCalibration.h"
using namespace cv;
using namespace Camera_Calibration;

int main()
{
    std::vector<cv::Point2d> line1, line2;
    std::shared_ptr<CameraCalibration> calibration = std::shared_ptr<CameraCalibration>(new CameraCalibration());
    Eigen::Vector3d polar1 = calibration->getPolar(calibration->concenteric_circle[3], calibration->poles[0]);
    Eigen::Vector3d polar2 = calibration->getPolar(calibration->concenteric_circle[3], calibration->poles[1]);
    Point2d pole1, pole2;
    calibration->Eigen2Point(calibration->poles[0], pole1);
    calibration->Eigen2Point(calibration->poles[0], pole1);
    line1.push_back(pole1);
    line1.push_back(pole2);
    line1.push_back(calibration->getPole(calibration->concenteric_circle[0], polar1));
    line1.push_back(calibration->getPole(calibration->concenteric_circle[1], polar1));
    line1.push_back(calibration->getPole(calibration->concenteric_circle[2], polar1));
    line2.push_back(calibration->getPole(calibration->concenteric_circle[0], polar2));
    line2.push_back(calibration->getPole(calibration->concenteric_circle[1], polar2));
    line2.push_back(calibration->getPole(calibration->concenteric_circle[2], polar2));

    cv::Vec4f line_para1, line_para2;
    cv::fitLine(line1, line_para1, cv::DIST_L2, 0, 1e-2, 1e-2);
    cv::fitLine(line2, line_para2, cv::DIST_L2, 0, 1e-2, 1e-2);

    std::cout << line_para1 << std::endl;
    std::cout << line_para2 << std::endl;
    double k_line1 = line_para1[1] / line_para1[0];
    double k_line2 = line_para2[1] / line_para2[0];

    double a1 = k_line1;
    double b1 = -1;
    double c1 = k_line1 * line_para1[2] + line_para1[3];
    Eigen::Vector3d l1;
    l1 << a1,
          b1,
          c1;
    double a2 = k_line2;
    double b2 = -1;
    double c2 = k_line2 * line_para2[2] + line_para2[3];
    Eigen::Vector3d l2;
    l2 << a2,
          b2,
          c2;

    cv::Point2d center = calibration->getLineIntersection(l1, l2);
    std::cout << center << std::endl;

    return 0;
}
