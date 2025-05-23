#include "start_detect.hpp"
#include <chrono>
#include "my_kalman.hpp"
#include <serial_port.hpp>
#include <sstream>
#include "gpio.hpp"
#include <PID.hpp>

void show_FPS(cv::Mat& frame)
{
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    double fps = 1.0 / elapsed_time.count();
    last_time = current_time;

    std::string fps_text = "FPS: " + std::to_string(fps);
    cv::putText(frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
}

void start_detect(int camera_index)
{
    Detect detector(camera_index);// 创建检测器对象

    //卡尔曼滤波器相关变量
    my_kalman_filter kalman_filter;// 卡尔曼滤波器对象
    cv::Point2f kalman_center = cv::Point2f(frame_width + 1, frame_height + 1);// 存储预测的中心点

    SerialPort serial("/dev/ttyCH341USB0", 115200);//串口通讯
    if (!serial.openPort()) 
    {
        std::cerr << "串口打开失败！" << std::endl;
        return;
    }

    gpiod_line* line = GPIO_init();// 初始化GPIO引脚

    PID pidx(0, 0, 0);
    PID pidy(0, 0, 0);

    // PID参数滑动条
    // 全局变量映射到 PID 参数
    int kp_sliderx = 3, ki_sliderx = 0, kd_sliderx = 3, alpha_sliderx = 5;
    int kp_slidery = 3, ki_slidery = 0, kd_slidery = 5, alpha_slidery = 5;
    cv::namedWindow("PID Tuning", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Kpx", "PID Tuning", &kp_sliderx, 1000);
    cv::createTrackbar("Kix", "PID Tuning", &ki_sliderx, 1000);
    cv::createTrackbar("Kdx", "PID Tuning", &kd_sliderx, 1000);
    cv::createTrackbar("Alphax", "PID Tuning", &alpha_sliderx, 100); // 0.0 ~ 1.0
    cv::createTrackbar("Kpy", "PID Tuning", &kp_slidery, 1000);
    cv::createTrackbar("Kiy", "PID Tuning", &ki_slidery, 1000);
    cv::createTrackbar("Kdy", "PID Tuning", &kd_slidery, 1000);
    cv::createTrackbar("Alphay", "PID Tuning", &alpha_slidery, 100); // 0.0 ~ 1.0

    //最终检测结果
    detect_return final;
    while(true)
    {
        final = detector.detectRedColor();// 检测红色物体

        if(final.measured_center != cv::Point2f(frame_width + 1, frame_height + 1) && gpiod_line_get_value(line)) // 如果测量的中心点不是初始化值,还有检测pin13
        {
            cv::circle(final.result, final.measured_center, 5, cv::Scalar(255, 0, 0), -1); // Draw center point
            std::cout << "measured_center: (" << final.measured_center.x << ", " << final.measured_center.y << ")" << std::endl;
            kalman_center = kalman_filter.get_kalman_filter(final.measured_center); // 使用卡尔曼滤波器预测中心点
            cv::circle(final.result, kalman_center, 5, cv::Scalar(0, 0, 255), -1);//红色点为卡尔曼滤波器预测的中心点
            std::cout << "kalman_center: (" << kalman_center.x << ", " << kalman_center.y << ")" << std::endl;
            std::cout << std::endl;

            //进行PID控制
            pidx.update(kp_sliderx / 1000.0f, ki_sliderx / 1000.0f, kd_sliderx / 1000.0f, alpha_sliderx / 100.0f);// 更新PID参数
            //kp初步测试3，即0.003//ki 0//kd 3//alpha 5
            pidy.update(kp_slidery / 1000.0f, ki_slidery / 1000.0f, kd_slidery / 1000.0f, alpha_slidery / 100.0f);// 更新PID参数
            cv::Point2f setpoint = cv::Point2f(frame_width / 2, frame_height / 2); // 设置目标点为图像中心

            float outputx = pidx.compute(setpoint.x, kalman_center.x); // 计算PID输出
            float outputy = pidy.compute(setpoint.y, kalman_center.y); // 计算PID输出

            //发送中心信息 
            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), "C%.2f, %.2f\n", outputx, outputy);
            serial.writeData(buffer, len);
        }

        show_FPS(final.result); // 显示FPS
        cv::imshow("Detected Red Color", final.result);// 显示处理后的图像

        if (cv::waitKey(1) == 'q') break;
    }

    serial.closePort();//关闭串口
}

// void start_detect(int camera_index)
// {
//     Detect detector(camera_index);

//     //最终检测结果
//     cv::Mat final;
//     while(true)
//     {
//         final = detector.detectRedColor();

//         show_FPS(final); // 显示FPS

//         cv::imshow("Detected Red Color", final);// 显示处理后的图像

//         if (cv::waitKey(1) == 'q') break;
//     }
// }