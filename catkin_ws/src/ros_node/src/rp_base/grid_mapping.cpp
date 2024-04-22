#include "grid_mapping.h"

  // maps so that rows/2 and cols/2 map in 0,0 in the world
void GridMapping::resize(int rows, int cols, float res) {
    _w2g.setIdentity();
    float inv_res=1./res;
    _w2g.linear() << inv_res, 0, 0, -inv_res;
    _w2g.translation() << cols*0.5, rows*0.5;
    _g2w=_w2g.inverse();
    _dw2g=_w2g.linear();
    _dg2w=_g2w.linear();
  }

  // maps so that col0 and row0 map in the upper left corner
  void GridMapping::resize(float res, const Eigen::Vector2f& upper_left) {
    _g2w.setIdentity();
    _g2w.linear() << res, 0, 0, -res;
    _g2w.translation() =upper_left;
    _w2g=_g2w.inverse();
    _dw2g=_w2g.linear();
    _dg2w=_g2w.linear();
  }
