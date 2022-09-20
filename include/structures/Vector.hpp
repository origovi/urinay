#pragma once

#include "structures/Point.hpp"

class Vector : public Point {
 private:
 public:
  Vector(const Point &a, const Point &b);
  inline double dot(const Vector &v) const;
  double angle(const Vector &v) const;
  static bool pointBehind(const Point &futPos, const Point &actPos, const Vector &dir);
  Vector rotClock() const;
  Vector rotCounterClock() const;
};