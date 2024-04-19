#pragma once
#include <vector>
#include <cmath>
#include <Eigen/Core>

using Eigen::Vector2f;
using Eigen::Vector2i;

template <typename CellType_>
struct Grid_ {
  using CellType=CellType_;
  using ContainerType=std::vector<CellType>;
  int rows;
  int cols;
  ContainerType cells;
  Grid_(int r=0, int c=0):
    rows(r),
    cols(c),
    cells(r*c){}

  void resize(int new_r, int new_c) {
    if (new_r==rows && new_c==cols)
      return;
    rows=new_r;
    cols=new_c;
    cells.resize(rows*cols);
  }
  
  inline CellType& at(int r, int c){
    return cells[r*cols+c];
  };

  inline const CellType& at(int r, int c) const {
    return cells[r*cols+c];
  };

  inline bool inside(int r, int c) const {
    return r>=0 && r<rows && c>=0 && c<cols;
  }

  // float accessors with interpolation, it does not return a writable cell
  inline bool inside(const Eigen::Vector2f& px) const {
    return px.x()>=1 && px.x()<cols-1 && px.y()>=1 && px.y()<rows-1;
  }

  /*https://en.wikipedia.org/wiki/Bilinear_interpolation*/
  inline const CellType at(const Eigen::Vector2f& px) const {
    const float r=px.y();
    const float c=px.x();
    float dr=r-floor(r);
    float dc=c-floor(c);
    int r0=r-dr;
    int c0=c-dc;
    const CellType& f00=at(r0,c0);
    const CellType& f10=at(r0+1,c0);
    const CellType& f01=at(r0,c0+1);
    const CellType& f11=at(r0+1,c0+1);
    const CellType a00=f00;
    const CellType a10=f10-f00;
    const CellType a01=f01-f00;
    const CellType a11=f11-f10-f01+f00;
    return a00+a10*dr+a01*dc+a11*dr*dc;
  }
  
   
  inline std::pair<int, int> ptr2idx(const CellType* c) const {
      const int offset=c-&cells[0];
      return std::make_pair(offset/cols, offset%cols);
  }

  // calculates the squared distance between two cells
  // using the pointer difference
  inline int distance2(const CellType&c1,
                       const CellType&c2) const {
        auto c1_pos=ptr2idx(&c1);
        auto c2_pos=ptr2idx(&c2);
        int dr=c1_pos.first-c2_pos.first;
        int dc=c1_pos.second-c2_pos.second;
        return dr*dr+dc*dc;
  }

  void fill (const CellType& c) {
    std::fill(cells.begin(), cells.end(), c);
  }

  void rowDerivative(Grid_<CellType>& dest) const {
    dest.resize(rows, cols);
    for (int r=1; r<rows-1; ++r)
      for (int c=1; c<cols-1; ++c)
        dest.at(r,c)=at(r+1,c)-at(r-1,c);
  }

  void colDerivative(Grid_<CellType>& dest) const {
    dest.resize(rows, cols);
    for (int r=1; r<rows-1; ++r)
      for (int c=1; c<cols-1; ++c)
        dest.at(r,c)=at(r,c+1)-at(r,c-1);
  }

  
  template <typename OtherCellType_>
  void copyTo(Grid_<OtherCellType_>& dest) const {
    dest.resize(rows, cols);
    for (size_t i=0; i<cells.size(); ++i) {
      dest.cells[i]=cells[i];
    }
  }
};

