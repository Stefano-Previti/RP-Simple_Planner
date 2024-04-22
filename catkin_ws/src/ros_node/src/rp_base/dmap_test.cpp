#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "dmap.h"
#include "grid_mapping.h"
#include "draw_helpers.h"
#include <sys/time.h> // for timing

using namespace std;

using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Isometry2f;


int main(int argc, char** argv) {
  if (argc<4) {
    std::cout << "usage " << argv[0] << " <rows> <cols> <fill_in>" << std::endl;
    return -1;
  }
  
  int rows=atoi(argv[1]);
  int cols=atoi(argv[2]);
  float fill_in=atof(argv[3]);
  
  float resolution=0.1;
  float damping = 1;
  
  DMap dmap(rows, cols);
  int size=rows*cols;
  int num_obstacles=size*fill_in;
  std::vector<std::pair<int,int>> obstacles(num_obstacles);
  for (auto& o: obstacles) {
    o.first=drand48()*rows;
    o.second=drand48()*cols;
  }

  Canvas canvas;
  int dmax_now=1;
  while (1) {
    int dmax2=dmax_now*dmax_now;
    dmap.clear();
    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    int ops = dmap.compute(obstacles, dmax2);
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "ops: " << ops << " time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;

    // check calculation of dmap
    Grid_<float> distances;
    dmap.copyTo(distances, dmax2);

    // compute the squared, and get the range
    float f_min=1e9;
    float f_max=0;
    for(auto& f: distances.cells) {
      f=sqrt(f);
      f_min=std::min(f, f_min);
      f_max=std::max(f, f_max);
    }
    float scale=255./(f_max-f_min);
    Grid_<uint8_t> image(distances.rows, distances.cols);
    for (size_t i=0; i<distances.cells.size(); ++i) {
      image.cells[i]=scale  * (distances.cells[i] - f_min);
    }

    // convert to image and show
    drawGrid(canvas, image);
    int ret=showCanvas(canvas, 0);
    switch(ret) {
    case 82: //up;
      dmax_now++;
      break;
    case 84: //down;
      dmax_now--;
      break;
    default:;
    }
    if (dmax_now<1)
      dmax_now=1;
  }
  
  // check derivatives
  // distances.colDerivative(distances_dc);
  // distances.rowDerivative(distances_dr);

  // ofstream os (filename);
  // ofstream osr (std::string("r_")+filename);
  // ofstream osc (std::string("c_")+filename);
  // for (float r=0; r<dmap.rows; r+=0.25) {
  //   for (float c=0; c<dmap.cols; c+=0.25){
  //     os << distances.at(Eigen::Vector2f(c,r)) << " ";
  //     osr << distances_dr.at(Eigen::Vector2f(c,r)) << " ";
  //     osc << distances_dc.at(Eigen::Vector2f(c,r)) << " ";
  //   }
  //   os << endl;
  //   osr << endl;
  //   osc << endl;
  // }
}
