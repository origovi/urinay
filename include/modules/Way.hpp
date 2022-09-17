#pragma once

#include <as_msgs/Tracklimits.h>

#include "modules/Visualization.hpp"
#include "utils/Definitions.hpp"
#include "utils/Params.hpp"
#include "utils/KDTree.hpp"

class Way {
 private:
  Params::Way params_;
  void filterTriangulation(TriangleSet &triangulation) const;

 public:
  Way(const Params::Way &params);
  void update(TriangleSet &triangulation, const Visualization &vis);
};