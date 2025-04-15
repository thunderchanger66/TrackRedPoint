#include "serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>

SerialPort::SerialPort(const std::string& device, int baudrate)
    : device_(device), baudrate_(baudrate), fd_(-1) {}

SerialPort::~SerialPort() {
    closePort();
}

bool SerialPort::openPort() {
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        perror("打开串口失败");
        return false;
    }

    fcntl(fd_, F_SETFL, 0);  // 设置为阻塞模式

    struct termios options;
    tcgetattr(fd_, &options);

    // 设置波特率
    speed_t speed;
    switch (baudrate_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B115200;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd_, TCSANOW, &options);

    return true;
}

void SerialPort::closePort() {
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const {
    return fd_ != -1;
}

int SerialPort::writeData(const char* data, size_t size) {
    if (fd_ == -1) return -1;
    return write(fd_, data, size);
}

int SerialPort::readData(char* buffer, int bufferSize) {
    if (fd_ == -1) return -1;
    return read(fd_, buffer, bufferSize);
}