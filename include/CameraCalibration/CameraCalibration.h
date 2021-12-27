//
// Created by 许斌 on 2021/12/26.
//

#ifndef CALIBRATION_CALIBRATION_H
#define CALIBRATION_CALIBRATION_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Camera_Calibration
{
class CameraCalibration
{
private:
    std::vector<cv::Point3f> objpoints;         // 世界坐标系下坐标
    std::vector<cv::Point2f> imgpoints;         // 图像坐标系下坐标
public:
    Eigen::RowVector3d getLineIntersection(Eigen::Vector3d l1, Eigen::Vector3d l2);     // 获取两条直线交点：l=x1 * x2
    Eigen::RowVector3d getTangent(Eigen::Matrix3d conic, Eigen::Vector3d polars);       // 获取二次曲线切线：l=Cx
};
}


#endif //CALIBRATION_CALIBRATION_H
