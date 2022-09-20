#pragma once

#include <cmath>
#include <iostream>

#include "structures/Node.hpp"

class Circle {
 private:
  Point center_, centerGlobal_;
  double radSq_;

 public:
  Circle(const Node &n0, const Node &n1, const Node &n2);
  bool containsNode(const Node &n) const;
  const Point &center() const;
  const Point &centerGlobal() const;
};