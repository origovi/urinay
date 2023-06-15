/**
 * @file Edge.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Edge class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Edge.hpp"

/* ----------------------------- Private Methods ---------------------------- */

uint64_t Edge::computeHash(const Node &n0, const Node &n1) {
  return (uint64_t(std::max(n0.id, n1.id)) << HASH_SHIFT_NUM) + std::min(n0.id, n1.id);
}

double Edge::computeLen(const Node &n0, const Node &n1) {
  return Point::dist(n0.pointGlobal(), n1.pointGlobal());
}

/* ----------------------------- Public Methods ----------------------------- */

Edge::Edge(const Node &n0, const Node &n1)
    : n0(n0), n1(n1), hash_(computeHash(n0, n1)), len(computeLen(n0, n1)) {}

bool Edge::operator==(const Edge &e) const {
  return this->hash_ == e.hash_;
}

bool Edge::operator!=(const Edge &e) const {
  return not(*this == e);
}

void Edge::updateLocal(const Eigen::Affine3d &tf) const {
  this->n0.updateLocal(tf);
  this->n1.updateLocal(tf);
}

Point Edge::midPoint() const {
  return (this->n0.point() + this->n1.point()) / 2.0;
}

Point Edge::midPointGlobal() const {
  return (this->n0.pointGlobal() + this->n1.pointGlobal()) / 2.0;
}

Vector Edge::normal() const {
  return Vector(this->n0.point(), this->n1.point()).rotClock();
}

std::ostream &operator<<(std::ostream &os, const Edge &e) {
  os << "E(" << e.n0 << ", " << e.n1 << ")";
  return os;
}