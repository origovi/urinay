#include "structures/Vector.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Vector::Vector(const Point &a, const Point &b)
    : Point(b - a) {}

double Vector::dot(const Vector &v) const {
  return this->x * v.x + this->y * v.y;
}

double Vector::angle(const Vector &v) const {
  double det = this->x * v.y - this->y * v.x;
  return atan2(det, this->dot(v));
}

bool Vector::pointBehind(const Point &futPos, const Point &actPos, const Vector &dir) {
  return Vector(actPos, futPos).dot(dir) < 0.0;
}

Vector Vector::rotClock() const {
  return Vector(Point(), Point(this->y, -this->x));
}

Vector Vector::rotCounterClock() const {
  return Vector(Point(), Point(-this->y, this->x));
}