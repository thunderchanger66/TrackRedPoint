#pragma once

#include <opencv2/opencv.hpp>

class my_kalman_filter
{
private:
    cv::Mat A;//状态转移矩阵
    cv::Mat H;//观测矩阵
    cv::Mat Q;//过程噪声协方差矩阵
    cv::Mat R;//观测噪声协方差矩阵
    cv::Mat P;//估计误差协方差矩阵
    cv::Mat x;//状态向量
    bool initialized;//是否初始化

    cv::Mat x_pred;//预测状态向量
    cv::Mat P_pred;//预测误差协方差矩阵
    cv::Mat z;//观测值

public:
    my_kalman_filter();
    void init(const cv::Point2f& initial_state);
    void predict();
    cv::Point2f update(const cv::Point2f& measurement); 
    cv::Point2f get_kalman_filter(const cv::Point2f& measurement);
};