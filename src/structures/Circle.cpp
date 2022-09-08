#include "structures/Circle.hpp"

/* ----------------------------- Private Methods ---------------------------- */

double Circle::distSq(const Node &n) const {
  return (this->x - n.x) * (this->x - n.x) + (this->y - n.y) * (this->y - n.y);
}

/* ----------------------------- Public Methods ----------------------------- */

Circle::Circle(const Node &p0, const Node &p1, const Node &p2) {
  const double ax = p1.x - p0.x;
  const double ay = p1.y - p0.y;
  const double bx = p2.x - p0.x;
  const double by = p2.y - p0.y;

  const double m = p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y;
  const double u = p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y;
  const double s = 1. / (2. * (ax * by - ay * bx));

  this->x = ((p2.y - p0.y) * m + (p0.y - p1.y) * u) * s;
  this->y = ((p0.x - p2.x) * m + (p1.x - p0.x) * u) * s;

  const double dx = p0.x - this->x;
  const double dy = p0.y - this->y;
  this->radSq = dx * dx + dy * dy;
}

bool Circle::containsNode(const Node &n) const {
  return distSq(n) < this->radSq;
}

geometry_msgs::Point Circle::center() const {
  geometry_msgs::Point res;
  res.x = this->x;
  res.y = this->y;
  res.z = 0.0;
  return res;
}
