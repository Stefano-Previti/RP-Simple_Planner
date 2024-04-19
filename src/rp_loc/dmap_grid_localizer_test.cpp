#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include "dmap_localizer.h"
#include "rp_base/draw_helpers.h"
#include "rp_base/grid_map.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

int main(int argc, char** argv) {
  if (argc<4) {
    std::cout << "usage " << argv[0] << "<map_image> <resolution> <dmax>" << std::endl;
    return -1;
  }

  std::string filename=argv[1];
  float resolution = atof(argv[2]);
  float dmax = atof(argv[3]);
  cerr << "parameters: " << endl;
  cerr << " resolution: " << resolution << endl;
  cerr << " dmax_influence: " << dmax << endl;

  // load the map
  GridMap grid_map(0,0,resolution);
  grid_map.loadFromImage(filename.c_str(), resolution);

  DMapLocalizer localizer;
  localizer.setMap(grid_map, dmax);
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;
  Grid_<uint8_t> image(distances.rows, distances.cols);
  
  // compute normalization of the DMAP
  float f_min=1e9;
  float f_max=0;
  for(auto& f: distances.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  float scale=255./(f_max-f_min);

  // copy the (normalized) distances of the DMAP
  for (size_t i=0; i<distances.cells.size(); ++i) {
    image.cells[i]=scale  * (distances.cells[i] - f_min);
  }

  // extract the obstacles from the gmap to use as measurements
  std::vector<Vector2f> obstacles;
  for (int r=0; r<grid_map.rows; ++r)
    for (int c=0; c<grid_map.cols; ++c)
      if (grid_map.at(r,c)<127) {
        Vector2f endpoint=grid_map.gm.grid2world(Vector2f(c,r));
        if (endpoint.norm()<20)
          obstacles.push_back(endpoint);
      }
  
  // add an obstacle image as alternative visualization
  Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
  obstacle_image.fill(0);
  drawGrid(canvas, obstacle_image);
  for (const auto& m: obstacles) {
    Vector2f m_hat_grid=localizer.gm.world2grid(m);
    drawCircle(canvas, m_hat_grid.cast<int>(), 3, 255);
  }
  // we draw with cv and get back the result, dirty...
  memcpy(&obstacle_image.cells[0], canvas.data, distances.rows*distances.cols);

  // initialize the localizer with a small offset
  Isometry2f X=Eigen::Isometry2f::Identity();
  X.linear()=Rotation2Df(0.3).matrix();
  X.translation()<< 0.5, 0.5;
  localizer.X=X;

  int current_key = 0;
  while (1) {
    switch (current_key) {
      case 0:
        drawGrid(canvas, image);
        break;
      case 1:
        drawGrid(canvas, obstacle_image);
        break;
      default:;
    }

    for (const auto& m: obstacles) {
      Vector2f m_hat=localizer.X*m;
      Vector2f m_hat_grid=localizer.gm.world2grid(m_hat);
      drawCircle(canvas, m_hat_grid.cast<int>(), 3, 127);
    }
    int key = showCanvas(canvas,0);
    if (key == 32) {
      current_key = (current_key + 1) % 2;
      continue;
    }

    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    localizer.localize(obstacles, 1);
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;
  }
}
