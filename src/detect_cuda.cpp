#include <iostream>
#include "detect_cuda.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>        // CUDA 基本支持
#include <opencv2/cudaimgproc.hpp>      // CUDA 图像处理功能，例如 cvtColor 等
#include <opencv2/cudaarithm.hpp>       // CUDA 算术操作，例如 bitwise_or/inRange

Detect::Detect(int camera_index)
{
    // 优先尝试 V4L2 打开摄像头（Linux 下通常更稳定）
    if (!cap.open(camera_index, cv::CAP_V4L2))
    {
        std::cerr << "Warning: Failed to open camera with V4L2, trying default backend..." << std::endl;

        // 退而求其次，使用默认方式
        cap.open(camera_index);
        if (!cap.isOpened())
        {
            std::cerr << "Error: Could not open camera with any backend." << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);   // 设置宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 设置高度

    lower_red1 = cv::Scalar(0, 100, 100);
    upper_red1 = cv::Scalar(10, 255, 255);  
    lower_red2 = cv::Scalar(160, 100, 100);
    upper_red2 = cv::Scalar(180, 255, 255);

    //kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //背景比较杂乱可以用下面这个
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    empty_value.result = cv::Mat(); // 初始化空返回对象
    empty_value.measured_center = cv::Point2f(frame_width + 1, frame_height + 1); // 初始化测量的中心点
}

//方法一
void getPoint(detect_return& return_value, const cv::Mat& mask_cpu, bool useContours)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_cpu, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Find the largest contour
    double maxArea = 0;
    std::vector<cv::Point> largestContour = {};

    //找到最大轮廓
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if(area > maxArea)
        {
            maxArea = area;
            largestContour = contour;
        }
    }

    cv::Rect bounding_box;// 存储最大轮廓的边界框
    if (maxArea > 500 && !largestContour.empty())
    {
        bounding_box = cv::boundingRect(largestContour);
        
        cv::rectangle(return_value.result, bounding_box, cv::Scalar(0, 255, 0), 2);

        //返回中心点
        return_value.measured_center = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2);
    }

    maxArea = 0.0; // Reset max_area for the next frame
    contours.clear(); // Clear contours for the next frame
    largestContour.clear();
}
//方法二
void getPoint(detect_return& return_value, const cv::Mat& mask_cpu)
{
    cv::Moments m = cv::moments(mask_cpu, true);
    if(m.m00 > 0)
        return_value.measured_center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00); // 计算重心
}

detect_return Detect::detectRedColor()
{
    cap >> frame;
    if (frame.empty())
    {
        std::cerr << "Error: Could not read frame." << std::endl;
        return empty_value;
    }

    // Upload frame to GPU
    frame_gpu.upload(frame, stream);

    cv::cuda::cvtColor(frame_gpu, hsv_gpu, cv::COLOR_BGR2HSV, 0, stream);
    cv::cuda::inRange(hsv_gpu, lower_red1, upper_red1, mask1_gpu, stream);
    cv::cuda::inRange(hsv_gpu, lower_red2, upper_red2, mask2_gpu, stream);
    // Combine the two masks (on GPU)
    cv::cuda::bitwise_or(mask1_gpu, mask2_gpu, mask_gpu, cv::noArray(), stream);

    mask_gpu.download(mask_cpu, stream);
    stream.waitForCompletion();  // 等待所有异步操作完成

    //cv::imshow("mask", mask_cpu); // 显示掩膜图像

    // Perform morphological operations
    cv::morphologyEx(mask_cpu, mask_cpu, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_cpu, mask_cpu, cv::MORPH_CLOSE, kernel);

    // 返回对象
    detect_return return_value;
    //存储最大轮廓
    return_value.result = frame.clone(); // Clone the original frame for display
    return_value.measured_center = cv::Point2f(frame_width + 1, frame_height + 1); 
    getPoint(return_value, mask_cpu); // 获取最大轮廓的中心点

    return return_value;
}

Detect::~Detect()
{
    cap.release();
    cv::destroyAllWindows();
}