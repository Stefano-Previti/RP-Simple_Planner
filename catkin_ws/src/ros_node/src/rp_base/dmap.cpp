#include "dmap.h"
#include <queue>
#include <iostream>

using namespace std;
  
int DMap::compute(const std::vector<std::pair<int,int>>& obstacles, int d2_max) {
  //1 populate the frontier
  std::queue<DMapCell*> frontier;
    
  for (const auto& p:obstacles) {
    int r=p.first, c=p.second;

    if (!inside(r,c))
      continue;

    auto& cell = at(r,c);
    cell.parent=&cell;
    frontier.push(&cell);
  }

  //2. expand
  int steps=0;
  while (! frontier.empty()) {
    ++steps;
    // fetch the node
    auto node=frontier.front();
    frontier.pop();

    // fetch the position
    auto node_pos=ptr2idx(node);
    int node_r=node_pos.first;
    int node_c=node_pos.second;
    
    // fetch the parent of the node
    auto ancestor=node->parent;

    // visit all neighbors
    for (int rr=node_r-1; rr <= node_r+1; ++rr) {
      for (int cc=node_c-1; cc <= node_c+1; ++cc) {
        if (rr==node_r && cc==node_c)
          continue;
        if (!inside(rr,cc))
          continue;
        auto& child=at(rr,cc);

        int d2_ancestor=distance2(child, *ancestor);
        if (d2_ancestor>d2_max) {
          continue;
        }
        if (! child.parent
            || d2_ancestor < distance2(child, *(child.parent)) ) {
          child.parent=ancestor;
          frontier.push(&child);
        }
        
      }
    }
  }
  return steps;
}

void DMap::clear() {
  DMapCell empty;
  fill(empty);
}
