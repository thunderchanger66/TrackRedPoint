#include <iostream>
#include "detect_cuda.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>        // CUDA 基本支持
#include <opencv2/cudaimgproc.hpp>      // CUDA 图像处理功能，例如 cvtColor 等
#include <opencv2/cudaarithm.hpp>       // CUDA 算术操作，例如 bitwise_or/inRange

Detect::Detect(int camera_index)
{
    cap.open(camera_index);
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
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

    //查找轮廓
    cv::findContours(mask_cpu, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //找到最大轮廓
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if(area > max_area)
        {
            max_area = area;
            largest_contour = contour;
        }
    }

    //存储最大轮廓
    return_value.result = frame.clone(); // Clone the original frame for display
    return_value.measured_center = cv::Point2f(frame_width + 1, frame_height + 1);
    if (max_area > 500 && !largest_contour.empty())
    {
        bounding_box = cv::boundingRect(largest_contour);
        
        cv::rectangle(return_value.result, bounding_box, cv::Scalar(0, 255, 0), 2);

        //返回中心点
        return_value.measured_center = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2);
    }

    max_area = 0.0; // Reset max_area for the next frame
    contours.clear(); // Clear contours for the next frame
    largest_contour.clear(); 

    return return_value;
}

Detect::~Detect()
{
    cap.release();
    cv::destroyAllWindows();
}