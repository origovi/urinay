#include "structures/Edge.hpp"

/* ----------------------------- Private Methods ---------------------------- */

size_t Edge::computeHash(const Node &n0, const Node &n1) {
  size_t max = std::max(n0.id, n1.id);
  size_t min = std::min(n0.id, n1.id);
  return max * 1e6 + min;
}

/* ----------------------------- Public Methods ----------------------------- */

Edge::Edge(const Node &n0, const Node &n1) : n0(n0), n1(n1), hash(computeHash(n0, n1)) {}

bool Edge::operator==(const Edge &e) const {
  return (e.n0 == this->n0 and e.n1 == this->n1) or (e.n1 == n0 and e.n0 == this->n1);
}

std::ostream &operator<<(std::ostream &os, const Edge &e) {
  os << "E(" << e.n0 << ", " << e.n1 << ")";
  return os;
}