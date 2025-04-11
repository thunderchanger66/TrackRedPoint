#include "gpio.hpp"
#include <iostream>

gpiod_line* GPIO_init()//只用检测pin13是否为低，若为低则停止检测
{
    const char* chipname = "gpiochip0";  // GPIO 芯片名
    unsigned int line_num = 122;         // 对应板子 PIN13 的 GPIO 号（根据需要确认）

    // 打开 GPIO 芯片
    gpiod_chip* chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "无法打开 GPIO 芯片: " << chipname << std::endl;
        return nullptr;
    }

    // 获取 GPIO 行（即引脚）
    gpiod_line* line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "无法获取 GPIO 引脚: " << line_num << std::endl;
        gpiod_chip_close(chip);
        return nullptr;
    }

    // 请求该引脚为输入模式
    if (gpiod_line_request_input(line, "gpio_input") < 0) {
        std::cerr << "无法请求 GPIO 为输入模式" << std::endl;
        gpiod_chip_close(chip);
        return nullptr;
    }

    return line;  // 返回这个引脚的句柄
}