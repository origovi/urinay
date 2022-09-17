#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include "utils/Definitions.hpp"
#include "structures/Triangle.hpp"

class DelaunayTri {
 private:
  static Triangle superTriangle(const std::vector<Node> &nodes);

 public:
  static TriangleSet compute(const std::vector<Node> &nodes);
};