#include "my_kalman.hpp"

my_kalman_filter::my_kalman_filter()
{
    A = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                  0, 1, 0, 1,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1);

    H = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                  0, 1, 0, 0);

    Q = cv::Mat::eye(4, 4, CV_32F) * 1e-1; 
    R = cv::Mat::eye(2, 2, CV_32F) * 1e-3;
    P = cv::Mat::eye(4, 4, CV_32F);
    x = cv::Mat::zeros(4, 1, CV_32F);
    initialized = false;
}

void my_kalman_filter::init(const cv::Point2f& initial_state)
{
    x.at<float>(0) = initial_state.x;
    x.at<float>(1) = initial_state.y;
    x.at<float>(2) = 0; // 初始速度为0
    x.at<float>(3) = 0; // 初始速度为0
    initialized = true;
}

void my_kalman_filter::predict()
{
    x_pred = A * x; // 预测状态
    P_pred = A * P * A.t() + Q; // 预测误差协方差矩阵
}

cv::Point2f my_kalman_filter::update(const cv::Point2f& measurement)
{
    z = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y); // 观测值

    cv::Mat y = z - H * x_pred; // 观测残差
    cv::Mat S = H * P_pred * H.t() + R; // 残差协方差矩阵
    cv::Mat K = P_pred * H.t() * S.inv(); // 卡尔曼增益

    x = x_pred + K * y; // 更新状态
    P = (cv::Mat::eye(4, 4, CV_32F) - K * H) * P_pred; // 更新误差协方差矩阵

    return cv::Point2f(x.at<float>(0), x.at<float>(1)); // 返回更新后的状态
}

cv::Point2f my_kalman_filter::get_kalman_filter(const cv::Point2f& measurement)
{
    if (!initialized)
    {
        init(measurement);
        return measurement; // 如果未初始化，则返回测量值
    }
    predict(); // 预测
    return update(measurement); // 更新并返回状态
}