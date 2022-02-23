//
// Created by 许斌 on 2022/2/23.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
using namespace std;

void DrawConcentricCircle(cv::Mat mask, const cv::Point2i &center, int radius1, int radius2, const cv::Scalar &color, int thickness, int linetype);

int main()
{
    cv::Mat src = cv::Mat(9000, 9000, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat result = src.clone();
    for(int x = 0; x < 7; x++)
    {
        for (int y = 0; y < 7; y++)
        {
            DrawConcentricCircle(result, cv::Point(x * 1200 + 900, y * 1200 + 900), 300, 500, cv::Scalar(0, 0, 0),-10, 16);

        }
    }
    imshow("result", result);
    cv::imwrite("/Users/xubin/Desktop/calibration/image/target.jpg", result);
//    cv::waitKey(0);
    system("pause");
    return 0;
}

// 绘制同心圆
void DrawConcentricCircle(cv::Mat mask, const cv::Point2i &center, int radius1,int radius2, const cv::Scalar &color, int thickness,int linetype)
{
    // 创建画布
    cv::Mat canvas = cv::Mat::zeros(mask.size(), CV_8UC1);

    // 计算内径和外径
    int inradius = min(radius1, radius2);
    int outradius = max(radius1, radius2);

    // 分情况讨论
    // 当thickness大于0时，绘制的是两个圆型线条组成的同心圆，不需填充
    if (thickness > 0)
    {
        cv::circle(mask, center, outradius, color, thickness, linetype);
        cv::circle(mask, center, inradius, color, thickness, linetype);
    }
        // 当thickness小于0，一般为-1，绘制的是填充同心圆，内圆不能有填充色
    else {
        cv::circle(canvas, center, outradius, cv::Scalar(255), -1, linetype);
        cv::circle(canvas, center, inradius, cv::Scalar(0), -1, linetype);
        int row = mask.rows;
        int col = mask.cols;
        for (int i = 0; i < row; ++i)
        {
            for (int j = 0; j < col; ++j)
            {
                uchar *m = canvas.ptr<uchar>(i);
                if (m[j] == 255)
                {
                    mask.at<cv::Vec3b>(i, j)[0] = static_cast<uchar>(color[0]);
                    mask.at<cv::Vec3b>(i, j)[1] = static_cast<uchar>(color[1]);
                    mask.at<cv::Vec3b>(i, j)[2] = static_cast<uchar>(color[2]);
                }

            }
        }
    }
}



