#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "my_kalman.hpp"

class Detect
{
private:
    cv::VideoCapture cap;//摄像头对象
    cv::Scalar lower_red1, upper_red1, lower_red2, upper_red2;// 红色范围的HSV值

    cv::Mat frame, hsv, mask1, mask2, mask;// 图像处理所需的变量

    cv::Mat kernel;// 形态学操作的内核

    std::vector<std::vector<cv::Point>> contours;// 存储轮廓的向量

    cv::Mat result;// 存储处理后的图像

    double max_area = 0.0;// 最大轮廓面积
    std::vector<cv::Point> largest_contour;// 存储最大轮廓的点集

    cv::Rect bounding_box;// 存储最大轮廓的边界框
    cv::Point2f measured_center;// 存储最大轮廓的中心点(测量值)

    //卡尔曼滤波器相关变量
    my_kalman_filter kalman_filter;// 卡尔曼滤波器对象
    cv::Point2f kalman_center;// 存储预测的中心点

    // 设置摄像头的分辨率
    int width = 640;   // 设置宽度
    int height = 480;   // 设置高度

public:
    Detect(int camera_index = 0);

    cv::Mat detectRedColor(); // 检测红色物体

    ~Detect();
};
