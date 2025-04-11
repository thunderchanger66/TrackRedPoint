#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect.hpp"

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
}

cv::Mat Detect::detectRedColor()
{
    cap >> frame;
    if (frame.empty())
    {
        std::cerr << "Error: Could not read frame." << std::endl;
        return cv::Mat();
    }

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    mask = mask1 | mask2;

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    //查找轮廓
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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

    //存储最大轮廓,执行卡尔曼滤波器
    result = frame.clone();
    if (max_area > 500 && !largest_contour.empty())
    {
        bounding_box = cv::boundingRect(largest_contour);

        measured_center = cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2);
        cv::circle(result, measured_center, 5, cv::Scalar(255, 0, 0), -1); // Draw center point
        std::cout << "measured_center: (" << measured_center.x << ", " << measured_center.y << ")" << std::endl;

        kalman_center = kalman_filter.get_kalman_filter(measured_center); // 使用卡尔曼滤波器预测中心点
        cv::circle(result, kalman_center, 5, cv::Scalar(0, 0, 255), -1);//红色点为卡尔曼滤波器预测的中心点
        std::cout << "kalman_center: (" << kalman_center.x << ", " << kalman_center.y << ")" << std::endl;
        std::cout << std::endl;

        cv::rectangle(result, bounding_box, cv::Scalar(0, 255, 0), 2);
    }

    max_area = 0.0; // Reset max_area for the next frame
    contours.clear(); // Clear contours for the next frame
    largest_contour.clear(); 

    //cv::imshow("Mask", mask);// 显示掩膜图像

    return result;
}

Detect::~Detect()
{
    cap.release();
    cv::destroyAllWindows();
}