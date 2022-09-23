#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "structures/Circle.hpp"
#include "structures/Edge.hpp"
#include "structures/Node.hpp"

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

  /**
 * @brief Compares two triangles, two triangles will be equal when the vertices
 * are the same (the ids are checked).
 * 
 * @param t 
 * @return true 
 * @return false 
 */
  bool operator==(const Triangle &t) const;
  bool operator!=(const Triangle &t) const;
  bool containsNode(const Node &n) const;
  bool circleContainsNode(const Node &n) const;
  bool anyNodeInSuperTriangle() const;
  bool containsEdge(const Edge &e) const;
  std::array<double, 3> angles() const;
  const Point &circumCenter() const;
  const Point &circumCenterGlobal() const;
  friend std::ostream &operator<<(std::ostream &os, const Triangle &t);
};

template <>
struct std::hash<Triangle> {
  size_t operator()(const Triangle &t) const {
    return t.hash_;
  }
};