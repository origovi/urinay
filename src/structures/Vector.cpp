/**
 * @file Vector.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Vector class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Vector.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Vector::Vector(const Point &a, const Point &b)
    : Point(b - a) {}

Vector::Vector(const double &x, const double &y)
    : Point(x, y) {}

double Vector::dot(const Vector &v) const {
  return this->x * v.x + this->y * v.y;
}

double Vector::angleWith(const Vector &v) const {
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