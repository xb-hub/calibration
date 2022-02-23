#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include "CameraCalibration/CameraCalibration.h"
using namespace cv;
using namespace Camera_Calibration;

//void getEllipseCenter(double a, double b, double c, double d, double e, double f, cv::Point2d& center)
//{
//    center.x = (2 * c * d - b * e) / (b * b - 4 * a * c);
//    center.y = (2 * a * e - b * d) / (b * b - 4 * a * c);
//}

int main()
{
    std::vector<cv::Point2d> line1, line2;
    std::shared_ptr<CameraCalibration> calibration = std::shared_ptr<CameraCalibration>(new CameraCalibration());
    for(int i = 0; i < calibration->concenteric_circle.size(); i++)
    {
        calibration->concenteric_pro_circle.push_back(calibration->getProjectConic(calibration->concenteric_circle[i]));
    }
    Eigen::Vector3d polar1 = calibration->getPolar(calibration->concenteric_pro_circle[0], calibration->poles[0]);
    Eigen::Vector3d polar2 = calibration->getPolar(calibration->concenteric_pro_circle[0], calibration->poles[1]);

    Point2d pole1, pole2;
    calibration->Eigen2Point(calibration->poles[0], pole1);
    calibration->Eigen2Point(calibration->poles[1], pole2);
    line1.push_back(pole1);
    line2.push_back(pole2);

    for (int i = 1; i < 4; ++i)
    {
        line1.push_back(calibration->getPole(calibration->concenteric_pro_circle[i], polar1));
        line2.push_back(calibration->getPole(calibration->concenteric_pro_circle[i], polar2));
    }

    Eigen::Vector3d l1, l2;
    l1 = calibration->getLineMatr(line1);
    l2 = calibration->getLineMatr(line2);

    cv::Point2d center = calibration->getLineIntersection(l1, l2);
    std::cout << center << std::endl;

    Eigen::Vector3d c{0, 0, 1};
    cv::Point2d cen;
    calibration->Eigen2Point(calibration->H_matrix * c, cen);
    std::cout << cen << std::endl;
    return 0;
}
