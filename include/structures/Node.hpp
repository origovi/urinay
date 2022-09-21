#pragma once

#include <geometry_msgs/Point.h>

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <as_msgs/Cone.h>

#include "structures/Point.hpp"

class Node {
 private:
  static const uint32_t SUPERTRIANGLE_BASEID = 999990;
  static uint32_t superTriangleNodeNum;
  const bool belongsToSuperTriangle_;
  Point point_;
  const Point pointGlobal_;
  Node(const double &x, const double &y);

 public:
  const uint32_t id;
  Node(const double &x, const double &y, const double &xGlobal, const double &yGlobal, const uint32_t &id);
  Node(const as_msgs::Cone &c);
  const double &x() const;
  const double &y() const;
  bool operator==(const Node &n) const;
  bool operator!=(const Node &n) const;
  static Node superTriangleNode(const double &x, const double &y);
  const bool &belongsToSuperTriangle() const;
  void updateLocal(const Eigen::Affine3d &tf);
  const Point &point() const;
  const Point &pointGlobal() const;
  double distSq(const Point &p) const;
  as_msgs::Cone cone() const;
  friend std::ostream &operator<<(std::ostream &os, const Node &n);
};