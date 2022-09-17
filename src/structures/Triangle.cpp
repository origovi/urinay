#include "structures/Triangle.hpp"

/* ----------------------------- Private Methods ---------------------------- */

size_t Triangle::computeHash(const Node &n0, const Node &n1, const Node &n2) {
  std::vector<size_t> ids = {n0.id, n1.id, n2.id};
  std::sort(ids.begin(), ids.end(), std::greater<size_t>());
  return ids[0] * 1e12 + ids[1] * 1e6 + ids[2];
}

/* ----------------------------- Public Methods ----------------------------- */

Triangle::Triangle(const Node &n0, const Node &n1, const Node &n2) : nodes{n0, n1, n2}, circle_(n0, n1, n2), edges{Edge(n0, n1), Edge(n1, n2), Edge(n0, n2)}, hash_(computeHash(n0, n1, n2)) {}
Triangle::Triangle(const Edge &e, const Node &n) : nodes{e.n0, e.n1, n}, circle_(e.n0, e.n1, n), edges{Edge(e.n0, e.n1), Edge(e.n1, n), Edge(e.n0, n)}, hash_(computeHash(e.n0, e.n1, n)) {}

/**
 * @brief Compares two triangles, two triangles will be equal when the vertices
 * are the same (the id and point are checked).
 * 
 * @param t 
 * @return true 
 * @return false 
 */
bool Triangle::operator==(const Triangle &t) const {
  std::vector<uint32_t> thisIds = {this->nodes[0].id, this->nodes[1].id, this->nodes[2].id};
  std::vector<uint32_t> tIds = {t.nodes[0].id, t.nodes[1].id, t.nodes[2].id};
  std::sort(thisIds.begin(), thisIds.end());
  std::sort(tIds.begin(), tIds.end());
  return thisIds[0] == tIds[0] and thisIds[1] == tIds[1] and thisIds[2] == tIds[2];
}

bool Triangle::containsNode(const Node &n) const {
  return this->nodes[0] == n or this->nodes[1] == n or this->nodes[2] == n;
}

bool Triangle::circleContainsNode(const Node &n) const {
  return circle_.containsNode(n);
}

bool Triangle::anyNodeInSuperTriangle() const {
  return nodes[0].belongsToSuperTriangle() or nodes[1].belongsToSuperTriangle() or nodes[2].belongsToSuperTriangle();
}

bool Triangle::containsEdge(const Edge &e) const {
  return e == this->edges[0] or e == this->edges[1] or e == this->edges[2];
}

const Point &Triangle::circumCenter() const {
  return circle_.center();
}

std::ostream &operator<<(std::ostream &os, const Triangle &t) {
  os << "T(" << t.nodes[0] << ", " << t.nodes[1] << ", " << t.nodes[2] << ")";
  return os;
}