#pragma once
#include "grid.h"
#include <limits>

using namespace std;

//distance map cell
//it has only the pointer to the parent 
struct DMapCell {
  DMapCell* parent=nullptr;
};

struct DMap: public Grid_<DMapCell> {

  DMap(int rows, int cols):
    Grid_<DMapCell>(rows,cols){}

  // computes the distance map by adding the occupied cells
  // int he obstacle array. The expansion is stopped when the
  // squared distance reaches d2_max
  int compute(const std::vector<std::pair<int,int>>& obstacles,
              int d2_max=std::numeric_limits<int>::max()); 
  
  void clear();

  // extracts the distance values (up to d2_max) and copies it
  // to the target
  template <typename OutputCellType>
  void copyTo(Grid_<OutputCellType>& dest, int d2_max=std::numeric_limits<int>::max()) {
    dest.resize(rows, cols);
    for (size_t i=0; i<cells.size(); ++i) {
      const auto& cell=cells[i];
      int d2=d2_max;
      if(cell.parent){
        d2=distance2(cell, *cell.parent);
      }
      dest.cells[i]=d2;
    }
  }
  
};
