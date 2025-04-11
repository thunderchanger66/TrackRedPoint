#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "my_kalman.hpp"

#define frame_width 640 // 摄像头分辨率宽度
#define frame_height 480 // 摄像头分辨率高度

typedef struct
{
    cv::Mat result; // 处理后的图像
    cv::Point2f measured_center; // 测量的中心点
} detect_return;

class Detect
{
private:
    cv::VideoCapture cap;//摄像头对象
    cv::Scalar lower_red1, upper_red1, lower_red2, upper_red2;// 红色范围的HSV值
    cv::Mat frame;// 当前帧

    cv::cuda::GpuMat frame_gpu, hsv_gpu, mask_gpu, mask1_gpu, mask2_gpu;// 图像处理所需的变量
    cv::cuda::Stream stream; // Optional CUDA stream for async processing

    cv::Mat kernel;// 形态学操作的内核
    cv::Mat mask_cpu;

    std::vector<std::vector<cv::Point>> contours;// 存储轮廓的向量

    double max_area = 0.0;// 最大轮廓面积
    std::vector<cv::Point> largest_contour;// 存储最大轮廓的点集

    cv::Rect bounding_box;// 存储最大轮廓的边界框

    // 设置摄像头的分辨率
    int width = frame_width;   //设置宽度
    int height = frame_height;   //设置高度

    //空返回对象
    detect_return empty_value;
    // 返回对象
    detect_return return_value;

public:
    Detect(int camera_index = 0);

    detect_return detectRedColor();

    ~Detect();
};
