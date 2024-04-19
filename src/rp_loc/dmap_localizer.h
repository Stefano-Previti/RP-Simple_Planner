#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rp_base/dmap.h"
#include "rp_base/grid_mapping.h"
#include "rp_base/grid_map.h"

using Eigen::Vector2f;
using Eigen::Isometry2f;

struct DMapLocalizer {
  // robot pose
  Isometry2f X=Isometry2f::Identity();
  
  // alg parameters: 
  float damping=1.;
  int inliers_min=10;
  float kernel_chi2=1.f;

  //internal stuff
  GridMapping gm;
  float resolution;
  Grid_<float> distances, distances_dr, distances_dc, distances_m;

  //initialization function, from obstacles
  void setMap(const std::vector<Eigen::Vector2f> obstacles,
              float res,
              float influence_range);

  // initialization function, from map
  void setMap(const GridMap& grid_map,
              float influence_range=2,
              uint8_t occ_threshold=127);
  
  bool localize(const std::vector<Vector2f>& measurements,
                int iterations=1);
  
  //bool localize(const LaserScan& scan, int iterations);
  
};
