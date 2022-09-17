#pragma once

#include "structures/Node.hpp"

class Edge {
 private:
  const size_t hash_;
  static size_t computeHash(const Node &n0, const Node &n1);
  static double computeLen(const Node &n0, const Node &n1);
  friend class std::hash<Edge>;

 public:
  const Node n0, n1;
  const double len;
  Edge(const Node &n0, const Node &n1);
  bool operator==(const Edge &e) const;
  friend std::ostream &operator<<(std::ostream &os, const Edge &e);
  Point midPoint() const;
};

template <>
struct std::hash<Edge> {
  size_t operator()(const Edge &e) const {
    return e.hash_;
  }
};