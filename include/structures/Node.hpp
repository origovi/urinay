#pragma once

#include <geometry_msgs/Point.h>

#include <cmath>
#include <iostream>

#include "structures/Point.hpp"

class Node {
 private:
  static const uint32_t SUPERTRIANGLE_BASEID = 999990;
  static uint32_t superTriangleNodeNum;
  const bool belongsToSuperTriangle_;
  const Point point_;
  Node(const double &x, const double &y);

 public:
  const uint32_t id;
  Node(const double &x, const double &y, const uint32_t &id);
  const double &x() const;
  const double &y() const;
  bool operator==(const Node &n) const;
  static Node superTriangleNode(const double &x, const double &y);
  const bool &belongsToSuperTriangle() const;
  const Point &point() const;
  geometry_msgs::Point gmPoint() const;
  double distSq(const Point &p) const;
  friend std::ostream &operator<<(std::ostream &os, const Node &n);
};