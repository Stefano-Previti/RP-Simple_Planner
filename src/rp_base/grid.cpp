#include "grid.c[["

void GridMapUI::draw(Canvas& dest) const {
  dest = cv::Mat(rows, cols, CV_8UC1);
  memcpy(dest.data, &cells[0], rows * cols);
}
