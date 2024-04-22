#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "grid_map.h"

using namespace std;

int main(int argc, char** argv) {
  if (argc<2) {
    cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
    return -1;
  }
  const char* filename=argv[1];
  float resolution=atof(argv[2]);

  cout << "Running " << argv[0]
       << " with arguments" << endl
       << "-filename:" << argv[1] << endl
       << "-resolution: " << argv[2] << endl;

  GridMap grid_map(0,0, 0.1);
  grid_map.loadFromImage(filename, resolution);
  Canvas canvas;
  Vector2f center(0.,0.);
  
  float alpha=0;
  while(1) {
    grid_map.draw(canvas);
    Vector2f direction;
    direction[0]=cos(alpha);
    direction[1]=sin(alpha);
    Vector2f dest;
    bool hit = grid_map.scanRay(dest, center, direction, 100);
    cerr << "hit: " << hit << endl;
    cerr << "origin: " << grid_map.gm.world2grid(center) << endl;
    cerr << "endpoint: " << grid_map.gm.world2grid(dest) << endl;
    drawLine(canvas,
             grid_map.gm.world2grid(center).cast<int>(),
             grid_map.gm.world2grid(dest).cast<int>(), 127);
    
    showCanvas(canvas, 0);
    
    alpha+=0.01;
    cerr << "alpha: " << alpha << endl;
  }
}
