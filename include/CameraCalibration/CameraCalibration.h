//
// Created by 许斌 on 2021/12/26.
//

#ifndef CALIBRATION_CALIBRATION_H
#define CALIBRATION_CALIBRATION_H
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Camera_Calibration
{
class CameraCalibration
{
private:
//    std::vector<cv::Point3f> objpoints;         // 世界坐标系下坐标
//    std::vector<cv::Point2f> imgpoints;         // 图像坐标系下坐标
//
//    Eigen::Vector3d center;

    Eigen::Matrix<double, 3, 4> intrinsic_matrix;
public:
    std::vector<Eigen::Matrix3d> concenteric_circle;
    std::vector<Eigen::Vector3d> poles;

    CameraCalibration();
    ~CameraCalibration();

    cv::Point2d getLineIntersection(Eigen::Vector3d l1, Eigen::Vector3d l2);    // 获取两条直线交点：l=x1 * x2
    Eigen::Vector3d getPolar(Eigen::Matrix3d conic, Eigen::Vector3d pole);          // 获取二次曲线极线：l=Cx
    cv::Point2d getPole(Eigen::Matrix3d conic, Eigen::Vector3d polar);          // 获取二次曲线极点：x=CL

    void Eigen2Point(Eigen::Vector3d pole, cv::Point2d& point);

};
}


#endif //CALIBRATION_CALIBRATION_H
