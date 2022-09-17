#pragma once

#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>
#include <array>
#include <algorithm>

#include "structures/Circle.hpp"
#include "structures/Node.hpp"
#include "structures/Edge.hpp"

class Triangle {
 private:
  friend class std::hash<Triangle>;
  const Circle circle_;
  const size_t hash_;
  static size_t computeHash(const Node &n0, const Node &n1, const Node &n2);

 public:
  const std::array<Node, 3> nodes;
  const std::array<Edge, 3> edges;
  Triangle(const Node &n0, const Node &n1, const Node &n2);
  Triangle(const Edge &e, const Node &n);
  bool operator==(const Triangle &t) const;
  bool containsNode(const Node &n) const;
  bool circleContainsNode(const Node &n) const;
  bool anyNodeInSuperTriangle() const;
  bool containsEdge(const Edge &e) const;
  const Point &circumCenter() const;
  friend std::ostream &operator<<(std::ostream &os, const Triangle &t);
};

template <>
struct std::hash<Triangle> {
  size_t operator()(const Triangle &t) const {
    return t.hash_;
  }
};