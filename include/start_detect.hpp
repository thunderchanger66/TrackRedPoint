#pragma once

//#include "detect.hpp"
#include "detect_cuda.hpp"

void show_FPS(cv::Mat& frame);

void start_detect(int camera_index = 0);