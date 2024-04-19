#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include "dmap_localizer.h"
#include "rp_base/draw_helpers.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

int main(int argc, char** argv) {
  if (argc<5) {
    std::cout << "usage " << argv[0] << " <num_obstacles> <range> <resolution> <dmax>" << std::endl;
    return -1;
  }

  int num_obstacles=atoi(argv[1]);
  float range=atof(argv[2]);
  float resolution = atof(argv[3]);
  float dmax = atof(argv[4]);
  cerr << "parameters: " << endl;
  cerr << " obstacles: " << num_obstacles << endl;
  cerr << " range: " << range << endl;
  cerr << " resolution: " << resolution << endl;
  cerr << " dmax_influence: " << dmax << endl;

  // TODO #1: generate random obstacles
  std::vector<Vector2f> obstacles;
  obstacles.reserve(num_obstacles);
  for (int i=0; i<num_obstacles; ++i) {
    obstacles.push_back(Vector2f::Random()*range);
  }
  cerr << "obstacles" << endl;

  // TODO #2:
  // - construct the localizer using the obstacles
  // - compute the distance map (call the setMap method)
  DMapLocalizer localizer;
  localizer.setMap(obstacles, resolution, dmax);
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;
  Grid_<uint8_t> image(distances.rows, distances.cols);
  const auto& distances_dr = localizer.distances_dr;
  Grid_<uint8_t> image_dr(distances_dr.rows, distances_dr.cols);
  const auto& distances_dc = localizer.distances_dc;
  Grid_<uint8_t> image_dc(distances_dc.rows, distances_dc.cols);
  const auto& distances_m = localizer.distances_m;
  Grid_<uint8_t> image_m(distances_m.rows, distances_m.cols);
  
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

  // TODO #4: compute normalization of the ROW DERIVATIVE
  f_min=1e9;
  f_max=-1e9;
  for(auto& f: distances_dr.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  scale=255./(f_max-f_min);
  // TODO #5: copy the (normalized) distances of the ROW DERIVATIVE
  for (size_t i=0; i<distances_dr.cells.size(); ++i) {
    image_dr.cells[i]=scale  * (distances_dr.cells[i] - f_min);
  }

  // TODO #7: compute normalization of the COL DERIVATIVE
  f_min=1e9;
  f_max=-1e9;
  for(auto& f: distances_dc.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  scale=255./(f_max-f_min);
  // TODO #8: copy the (normalized) distances of the COL DERIVATIVE
  for (size_t i=0; i<distances_dc.cells.size(); ++i) {
    image_dc.cells[i]=scale  * (distances_dc.cells[i] - f_min);
  }

  // TODO #11: compute normalization of the DERIVATIVE MAGNITUDE
  f_min=1e9;
  f_max=-1e9;
  for(auto& f: distances_m.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  scale=255./(f_max-f_min);
  // TODO #12: copy the (normalized) distances of the DERIVATIVE MAGNITUDE
  for (size_t i=0; i<distances_m.cells.size(); ++i) {
    image_m.cells[i]=scale  * (distances_m.cells[i] - f_min);
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
  X.translation()<< 1, 0.5;
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
      // TODO #6: case for ROW DERIVATIVE
      case 2:
        drawGrid(canvas, image_dr);
        break;
      // TODO #9: case for COL DERIVATIVE
      case 3:
        drawGrid(canvas, image_dc);
        break;
      // TODO #13: case for DERIVATIVE MAGNITUDE
      case 4:
        drawGrid(canvas, image_m);
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
      current_key = (current_key + 1) % 5;
      continue;
    }

    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    // TODO #3: use the localizer to perform one localization step
    localizer.localize(obstacles, 1);
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;
  }
}
