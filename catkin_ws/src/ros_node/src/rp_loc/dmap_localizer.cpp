#include "dmap_localizer.h"
#include <iostream>
#include <fstream>

using Matrix_2_3f = Eigen::Matrix<float, 2,3>;
using Matrix_1_2f = Eigen::Matrix<float, 1,2>;
using Matrix_1_3f = Eigen::Matrix<float, 1,3>;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

void DMapLocalizer::setMap(const std::vector<Eigen::Vector2f> obstacles,
                           float res,
                           float influence_range) {

  resolution=res;
  //1 compute the bounding box of the obstacles
  Vector2f lower_left(std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
  Vector2f upper_right(std::numeric_limits<float>::min(),
                       std::numeric_limits<float>::min());
  for (const auto& o: obstacles) {
    lower_left.x()=min(lower_left.x(), o.x());
    lower_left.y()=min(lower_left.y(), o.y());
    upper_right.x()=max(upper_right.x(), o.x());
    upper_right.y()=max(upper_right.y(), o.y());
  }
  Vector2f bbox=upper_right-lower_left;
  
  //2. construct a mapping that assignes the upper_left corner of the bbox
  //   to the pixel 0,0;
  gm.resize(res,Vector2f(lower_left.x(),upper_right.y()));
    
  //3. construct the map the obstacles in grid coordinates and construct the dmap
  int rows=ceil(bbox.y())/resolution+1;
  int cols=ceil(bbox.x())/resolution+1;
  std::vector<std::pair<int, int>> grid_obstacles;
  grid_obstacles.reserve(obstacles.size());
  for (const auto& o: obstacles){
    const auto px=gm.world2grid(o);
    grid_obstacles.push_back(std::make_pair(px.y(), px.x()));
  }

  // calculate the influence range, squared in pixels
  int dmax_2=pow(influence_range/res,2);
  DMap dmap(rows, cols);
  dmap.clear();
  int ops = dmap.compute(grid_obstacles, dmax_2);
  
  //compute from the dmap the distances
  dmap.copyTo(distances, dmax_2);

  // convert it from pixels (squared) to meters
  for (auto& f: distances.cells)
    f=sqrt(f)*resolution;

  // compute the derivatives
  distances.colDerivative(distances_dc);
  distances.rowDerivative(distances_dr);

  // TODO #10: compute the derivative magnitude
  distances_m.resize(rows, cols);
  for (int r=0; r<rows; ++r) {
    for (int c=0; c<cols; ++c) {
      Vector2f d(distances_dr.at(r,c), distances_dc.at(r,c));
      distances_m.at(r,c)= d.squaredNorm();
    }
  }

  // initialize the robot to be in the middle of the map
  X.setIdentity();
  X.translation()=0.5*(upper_right+lower_left);
}

// initialization function, from map
void DMapLocalizer::setMap(const GridMap& grid_map,
                           float influence_range,
                           uint8_t occ_threshold) {
  // extract the obstacles
  std::vector<Vector2f> obstacles;
  for (int r=0; r<grid_map.rows; ++r)
    for (int c=0; c<grid_map.cols; ++c)
      if (grid_map.at(r,c)<occ_threshold) {
        obstacles.push_back(grid_map.gm.grid2world(Vector2f(c,r)));
      }
  // use the default initializer
  setMap(obstacles, grid_map.resolution, influence_range);
}


// this is magic for now
bool DMapLocalizer::localize(const std::vector<Vector2f>& measurements,
                             int iterations) {
  int inliers=0;
  float chi2=0;
  for (int i=0; i<iterations; ++i) {
    Matrix3f H;
    Vector3f b;
    Matrix_2_3f J_icp;
    const Matrix2f J_gm = gm.world2gridDerivative();
    Matrix_1_2f J_dmap;
    Matrix_1_3f J;
    
    J_icp.block<2,2>(0,0).setIdentity();
    inliers=0;
    H.setZero();
    b.setZero();
    for (const auto& m: measurements) {
      Vector2f p_world = X*m;
      Vector2f p_grid=gm.world2grid(p_world);
      if (! distances.inside(p_grid))
        continue;
      float e=distances.at(p_grid);
      float e2=e*e;
      float lambda=1.f;
      if (e2>kernel_chi2) {
        lambda=kernel_chi2/sqrt(e2);
      }
        
      J_icp.col(2) << -p_world.y(), p_world.x();
      J_dmap.x()=distances_dc.at(p_grid);
      J_dmap.y()=distances_dr.at(p_grid);
      
      J = J_dmap * J_gm * J_icp;
      H += lambda*J.transpose()*J;
      b += lambda*J.transpose()*e;
      chi2+=e2;
      ++ inliers;
    }
    H+=Eigen::Matrix3f::Identity()*damping;
    Vector3f dx= H.ldlt().solve(-b);
    Isometry2f dX;
    dX.translation() << dx.x(), dx.y();
    dX.linear()=Rotation2Df(dx.z()).matrix();
    X=dX*X;
    Vector3f x;
    x.x() = X.translation().x();
    x.y() = X.translation().y();
    x.z()=Rotation2Df(X.linear()).angle();
    cerr << "iteration: " << i
         << " chi2: " << chi2
         << " inliers: " << inliers << "/" << measurements.size()
         << " x: " << x.transpose() << endl;
  }
  return inliers > inliers_min;

}
