#pragma once
#include <cstdint>
#include "grid.h"
#include "grid_mapping.h"
#include "draw_helpers.h"


// grid mapping class
struct GridMap: public Grid_<uint8_t> {
  GridMapping gm;
  float resolution;      // meters*pixel
  GridMap(int rows, int cols, float resolution);

  // loads a map from an image
  void loadFromImage(const char* filename, float resolution);

  bool scanRay(Vector2f& hit,
               const Vector2f& origin,
               const Vector2f& direction,
               const float max_range) const;

  float scanRay(const Vector2f& origin,
                 const Vector2f& direction,
                 const float max_range) const;
    
  void draw(Canvas& dest) const;
};
