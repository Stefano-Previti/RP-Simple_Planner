#pragma once
#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "grid.h"

using Canvas = cv::Mat;
using Eigen::Vector2i;

void drawGrid(Canvas& dest, const Grid_<uint8_t>& src);

// draws a line from p0 to p1
void drawLine(Canvas& dest, const Vector2i& p0, const Vector2i& p1, uint8_t color);
void drawCircle(Canvas& dest, const Vector2i& center, int radius, uint8_t color);
int showCanvas(Canvas& canvas, int timeout_ms);
