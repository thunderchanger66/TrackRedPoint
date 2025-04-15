cmake_minimum_required(VERSION 3.10.0)
project(greentrack VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET libgpiod)

add_executable(greentrack src/main.cpp src/my_kalman.cpp src/start_detect.cpp src/detect_cuda.cpp src/serial_port.cpp src/gpio.cpp src/PID.cpp)
target_link_libraries(greentrack ${OpenCV_LIBS})
target_link_libraries(greentrack PkgConfig::GPIOD)
