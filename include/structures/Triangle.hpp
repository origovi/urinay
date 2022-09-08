#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include "structures/Circle.hpp"
#include "structures/Node.hpp"
#include "structures/Edge.hpp"

class Triangle {
 private:
  friend class std::hash<Triangle>;
  const Circle circle;
  size_t hash;
  static size_t computeHash(const Node &n0, const Node &n1, const Node &n2);

 public:
  const Node n0, n1, n2;
  const Edge e0, e1, e2;
  Triangle(const Node &n0, const Node &n1, const Node &n2);
  Triangle(const Edge &e, const Node &n);
  bool operator==(const Triangle &t) const;
  bool containsNode(const Node &n) const;
  bool circleContainsNode(const Node &n) const;
  bool anyNodeInSuperTriangle() const;
  bool containsEdge(const Edge &e) const;
  std::vector<Edge> edges() const;
  geometry_msgs::Point circumCenter() const;
  friend std::ostream &operator<<(std::ostream &os, const Triangle &t);
};

template <>
struct std::hash<Triangle> {
  size_t operator()(const Triangle &t) const {
    return t.hash;
  }
};

#endif  // TRIANGLE_HPP
