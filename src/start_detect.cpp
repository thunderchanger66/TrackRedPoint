#include "start_detect.hpp"
#include <chrono>
#include "my_kalman.hpp"
#include "serial_port.hpp"
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

    PID pidy(0, 0, 0);
    // 全局变量映射到 PID 参数
    int kp_slider = 0, ki_slider = 0, kd_slider = 0, alpha_slider = 10;
    cv::namedWindow("PID Tuning", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Kp", "PID Tuning", &kp_slider, 1000);
    cv::createTrackbar("Ki", "PID Tuning", &ki_slider, 1000);
    cv::createTrackbar("Kd", "PID Tuning", &kd_slider, 1000);
    cv::createTrackbar("Alpha", "PID Tuning", &alpha_slider, 100); // 0.0 ~ 1.0

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
            //kp初步测试3，即0.003
            pidy.update(kp_slider / 1000.0f, ki_slider / 1000.0f, kd_slider / 1000.0f, alpha_slider / 100.0f);// 更新PID参数
            cv::Point2f setpoint = cv::Point2f(frame_width / 2, frame_height / 2); // 设置目标点为图像中心
            float outputy = pidy.compute(setpoint.y, kalman_center.y); // 计算PID输出

            //发送中心信息 
            std::stringstream ss;
            ss << "C" << 0 << ", " << outputy << "\n";
            serial.writeData(ss.str());
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