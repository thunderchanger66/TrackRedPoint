#pragma once

#include <string>

class SerialPort {
public:
    SerialPort(const std::string& device, int baudrate = 115200);
    ~SerialPort();

    bool openPort();
    void closePort();

    bool isOpen() const;

    int writeData(const std::string& data);
    int readData(char* buffer, int bufferSize);

private:
    std::string device_;
    int baudrate_;
    int fd_;
};