#pragma once
#include <Eigen/Core>
#include <vector>
#include "grid_map.h"

using Eigen::Isometry2f;

struct LaserScan {
  float
    range_min=0.1,
    range_max=10,
    angle_min=-M_PI/2,
    angle_max=M_PI/2;
  std::vector<float> ranges;

  LaserScan (float range_min=0.1,
             float range_max=10,
             float angle_min=-M_PI/2,
             float angle_max=M_PI/2,
             int ranges_num=180);

  void draw(Canvas& canevasso, const GridMap& grid_map, const Isometry2f& pose);

};
