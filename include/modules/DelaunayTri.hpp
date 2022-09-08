#ifndef DELAUNAYTRI_HPP
#define DELAUNAYTRI_HPP

#include <cmath>
#include <iostream>
#include <vector>

#include "structures/Triangle.hpp"

using TriangleSet = std::unordered_set<Triangle>;
using EdgeSet = std::unordered_set<Edge>;

class DelaunayTri {
 private:
  static Triangle superTriangle(const std::vector<Node> &nodes);

 public:
  static TriangleSet compute(const std::vector<Node> &nodes);
};

#endif  // DELAUNAYTRI_HPP
