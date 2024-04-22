#include "grid_map.h"
using namespace std;

GridMap::GridMap(int rows_, int cols_, float resolution_):
  Grid_<uint8_t>(rows_, cols_),
  resolution(resolution_)
{
  gm.resize(rows_, cols_, resolution);
}

bool GridMap::scanRay(Vector2f& hit,
                      const Vector2f& origin,
                      const Vector2f& direction,
                      const float max_range) const{
  float range = 0;
  while (range < max_range) {
    hit = origin + direction * range;
    Vector2f grid_endpoint = gm.world2grid(hit);
    int r=grid_endpoint.y();
    int c=grid_endpoint.x();
    range += resolution;
    if (!inside(r,c)) return false;
    uint8_t val = at(r,c);
    if (val < 127) {
      return true;
    }
  }
  return false;
}

float GridMap::scanRay(const Vector2f& origin,
                        const Vector2f& direction,
                        const float max_range) const {
  float range = 0;

  while (range < max_range) {
    Vector2f grid_endpoint = gm.world2grid(origin + direction * range);
    int r=grid_endpoint.y();
    int c=grid_endpoint.x();
    if (!inside(r,c))
      return max_range;
    if (at(r,c) < 127)
      return range;
    range +=resolution;
  }
  
  return max_range;
}

void GridMap::loadFromImage(const char* filename, float res) {
  resolution = res;
  cerr << "loading [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0) {
    throw std::runtime_error("unable to load image");
  }
  cv::Mat loaded_image;
  cv::cvtColor(m, loaded_image, cv::COLOR_BGR2GRAY);
  int size = loaded_image.rows * loaded_image.cols;
  resize(loaded_image.rows, loaded_image.cols);
  gm.resize(rows, cols, res);
  cerr << "transform: " << endl;
  cerr << gm._w2g.matrix() << endl;
  
  memcpy(&cells[0], loaded_image.data, size);
}

void GridMap::draw(Canvas& dest) const {
  drawGrid(dest, *this);
}
