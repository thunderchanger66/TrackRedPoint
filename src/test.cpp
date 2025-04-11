#include <gpiod.h>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>


int main() {
    const char* chipname = "gpiochip0";  // GPIO 芯片名
    unsigned int line_num = 122;         // 引脚编号（根据你的需要更改）

    // 打开 GPIO 芯片
    gpiod_chip* chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "无法打开 GPIO 芯片: " << chipname << std::endl;
        return EXIT_FAILURE;
    }

    // 获取 GPIO 行（即引脚）
    gpiod_line* line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "无法获取 GPIO 引脚: " << line_num << std::endl;
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }

    // 请求该引脚为输入模式
    if (gpiod_line_request_input(line, "gpio_input") < 0) {
        std::cerr << "无法请求 GPIO 为输入模式" << std::endl;
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }

    while (true) {
        int value = gpiod_line_get_value(line);
        std::cout << "GPIO状态: " << value << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 关闭资源
    gpiod_chip_close(chip);
    return EXIT_SUCCESS;
}
