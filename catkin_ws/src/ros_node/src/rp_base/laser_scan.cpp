#include "laser_scan.h"

LaserScan::LaserScan (float range_min,
                      float range_max,
                      float angle_min,
                      float angle_max,
                      int ranges_num)
{
  this->range_min= range_min;
  this->range_max= range_max;
  this->angle_min= angle_min;
  this->angle_max= angle_max;
  ranges.resize(ranges_num);
  std::fill(ranges.begin(), ranges.end(), range_max);
}


void LaserScan::draw(Canvas& canevasso,
                     const GridMap& grid_map,
                     const Isometry2f& pose) {

  Vector2f center_px=grid_map.gm.world2grid(pose.translation());
  float angle_increment = (angle_max-angle_min)/ranges.size();
    
  for (auto i=0; i<ranges.size(); ++i) {
    float beam_angle=angle_min+angle_increment*i;
    Vector2f d(cos(beam_angle)*ranges[i], sin(beam_angle)*ranges[i]);
    Vector2f ep = pose * d;
    Vector2f ep_px = grid_map.gm.world2grid(ep);
    drawLine(canevasso, center_px.cast<int>(), ep_px.cast<int>(), 90); 
  }
}
