#include "structures/Triangle.hpp"

/* ----------------------------- Private Methods ---------------------------- */

size_t Triangle::computeHash(const Node &n0, const Node &n1, const Node &n2) {
  std::vector<size_t> ids = {n0.id, n1.id, n2.id};
  std::sort(ids.begin(), ids.end(), std::greater<size_t>());
  return ids[0] * 1e12 + ids[1] * 1e6 + ids[2];
}

/* ----------------------------- Public Methods ----------------------------- */

Triangle::Triangle(const Node &n0, const Node &n1, const Node &n2) : n0(n0), n1(n1), n2(n2), circle(n0, n1, n2), e0(n0, n1), e1(n1, n2), e2(n0, n2), hash(computeHash(n0, n1, n2)) {}
Triangle::Triangle(const Edge &e, const Node &n) : n0(e.n0), n1(e.n1), n2(n), circle(e.n0, e.n1, n), e0(e.n0, e.n1), e1(e.n1, n), e2(e.n0, n), hash(computeHash(e.n0, e.n1, n)) {}

/**
 * @brief Compares two triangles, two triangles will be equal when the vertices
 * are the same (the id and point are checked).
 * 
 * @param t 
 * @return true 
 * @return false 
 */
bool Triangle::operator==(const Triangle &t) const {
  std::vector<uint32_t> thisIds = {this->n0.id, this->n1.id, this->n2.id};
  std::vector<uint32_t> tIds = {t.n0.id, t.n1.id, t.n2.id};
  std::sort(thisIds.begin(), thisIds.end());
  std::sort(tIds.begin(), tIds.end());
  return thisIds[0] == tIds[0] and thisIds[1] == tIds[1] and thisIds[2] == tIds[2];
}

bool Triangle::containsNode(const Node &n) const {
  return n0 == n or n1 == n or n2 == n;
}

bool Triangle::circleContainsNode(const Node &n) const {
  return circle.containsNode(n);
}

bool Triangle::anyNodeInSuperTriangle() const {
  return n0.belongsToSuperTriangle() or n1.belongsToSuperTriangle() or n2.belongsToSuperTriangle();
}

bool Triangle::containsEdge(const Edge &e) const {
  return e == e0 or e == e1 or e == e2;
}

std::vector<Edge> Triangle::edges() const {
  return {e0, e1, e2};
}

geometry_msgs::Point Triangle::circumCenter() const {
  return circle.center();
}

std::ostream &operator<<(std::ostream &os, const Triangle &t) {
  os << "T(" << t.n0 << ", " << t.n1 << ", " << t.n2 << ")";
  return os;
}