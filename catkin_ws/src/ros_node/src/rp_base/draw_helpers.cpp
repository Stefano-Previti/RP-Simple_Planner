#include "draw_helpers.h"

void drawGrid(Canvas& dest, const Grid_<uint8_t>& src) {
  dest = cv::Mat(src.rows, src.cols, CV_8UC1);
  memcpy(dest.data, &src.cells[0], src.rows * src.cols);
}

void drawLine(Canvas& dest, const Vector2i& p0, const Vector2i& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]),
           cv::Scalar(color, color, color), 1);
}

void drawCircle(Canvas& dest, const Vector2i& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius,
             cv::Scalar(color, color, color));
}

int showCanvas(Canvas& canvas, int timeout_ms) {
  cv::imshow("canvas", canvas);
  int key = cv::waitKey(timeout_ms);
  if (key == 27)  // exit on ESC
    exit(0);
  // cerr << "key" << key << endl;
  return key;
}
