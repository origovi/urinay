#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <cmath>
#include <iostream>

#include "structures/Node.hpp"

class Circle {
 private:
  double x, y, radSq;
  double distSq(const Node &n) const;

 public:
  Circle(const Node &n0, const Node &n1, const Node &n2);
  bool containsNode(const Node &n) const;
  geometry_msgs::Point center() const;
};

#endif  // CIRCLE_HPP
