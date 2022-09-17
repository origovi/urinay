#include "structures/Circle.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Circle::Circle(const Node &p0, const Node &p1, const Node &p2) {
  const double ax = p1.x() - p0.x();
  const double ay = p1.y() - p0.y();
  const double bx = p2.x() - p0.x();
  const double by = p2.y() - p0.y();

  const double m = p1.x() * p1.x() - p0.x() * p0.x() + p1.y() * p1.y() - p0.y() * p0.y();
  const double u = p2.x() * p2.x() - p0.x() * p0.x() + p2.y() * p2.y() - p0.y() * p0.y();
  const double s = 1. / (2. * (ax * by - ay * bx));

  this->point_.x = ((p2.y() - p0.y()) * m + (p0.y() - p1.y()) * u) * s;
  this->point_.y = ((p0.x() - p2.x()) * m + (p1.x() - p0.x()) * u) * s;

  const double dx = p0.x() - this->point_.x;
  const double dy = p0.y() - this->point_.y;
  this->radSq_ = dx * dx + dy * dy;
}

bool Circle::containsNode(const Node &n) const {
  return n.distSq(this->point_) < this->radSq_;
}

const Point &Circle::center() const {
  return this->point_;
}
