#pragma once

#include "structures/Point.hpp"

class Vector : public Point {
 public:
  Vector() = default;
  Vector(const Point &a, const Point &b);
  Vector(const double &x, const double &y);
  inline double dot(const Vector &v) const;
  double angle(const Vector &v) const;
  static bool pointBehind(const Point &futPos, const Point &actPos, const Vector &dir);
  Vector rotClock() const;
  Vector rotCounterClock() const;
};