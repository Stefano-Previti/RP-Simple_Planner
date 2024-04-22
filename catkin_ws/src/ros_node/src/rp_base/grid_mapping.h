#pragma once
#include <Eigen/Geometry>

struct GridMapping {
  Eigen::Transform<float,2,Eigen::Affine> _w2g, _g2w;
  Eigen::Matrix2f _dw2g, _dg2w;

  // maps so that rows/2 and cols/2 map in 0,0 in the world
  void resize(int rows, int cols, float res);
  
  // maps so that col0 and row0 map in the upper left corner
  void resize(float res, const Eigen::Vector2f& upper_left);
  
  inline Eigen::Vector2f world2grid(const Eigen::Vector2f& wp) const {
    return _w2g*wp; 
  }
  inline Eigen::Vector2f grid2world(const Eigen::Vector2f& gp) const {
    return _g2w*gp;
  }
  inline const Eigen::Matrix2f& world2gridDerivative() const {
    return _dw2g;
  }
  inline const Eigen::Matrix2f& grid2worldDerivative() const {
    return _dg2w;
  }
};
