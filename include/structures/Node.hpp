#ifndef NODE_HPP
#define NODE_HPP

#include <geometry_msgs/Point.h>

#include <cmath>
#include <iostream>

class Node {
 private:
  static const uint32_t SUPERTRIANGLE_BASEID = 999990;
  static uint32_t superTriangleNodeNum;
  const bool belongsToSuperTriangle_;
  Node(const double &x, const double &y);

 public:
  const double x, y;
  const uint32_t id;
  Node(const double &x, const double &y, const uint32_t &id);
  bool operator==(const Node &n) const;
  static Node superTriangleNode(const double &x, const double &y);
  const bool &belongsToSuperTriangle() const;
  geometry_msgs::Point gmPoint() const;
  friend std::ostream &operator<<(std::ostream &os, const Node &n);
};

#endif  // NODE_HPP
