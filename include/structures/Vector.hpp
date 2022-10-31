/**
 * @file Vector.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Vector class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include "structures/Point.hpp"

/**
 * @brief Represents a vector in 2D space, inherits from the Point class.
 * It provides all basic functions with vectors in 2D.
 */
class Vector : public Point {
 public:
  /**
   * @brief Construct a new Vector object.
   */
  Vector() = default;

  /**
   * @brief Construct a new Vector object from two Point(s).
   * 
   * @param[in] a 
   * @param[in] b 
   */
  Vector(const Point &a, const Point &b);

  /**
   * @brief Construct a new Vector object from the two coordinates.
   * 
   * @param[in] x 
   * @param[in] y 
   */
  Vector(const double &x, const double &y);

  /**
   * @brief Returns the dot product between the implicit Vector and \a v.
   * 
   * @param[in] v 
   */
  inline double dot(const Vector &v) const;

  /**
   * @brief Returns the angle that the implicit Vector makes with \a v.
   * 
   * @param[in] v 
   */
  double angleWith(const Vector &v) const;

  /**
   * @brief Checks if the Point \a actPos is behind \a futPos along the direction \a dir.
   * 
   * @param[in] futPos 
   * @param[in] actPos 
   * @param[in] dir 
   */
  static bool pointBehind(const Point &futPos, const Point &actPos, const Vector &dir);
  
  /**
   * @brief Returns a Vector rotated clockwise 90 deg.
   */
  Vector rotClock() const;

  /**
   * @brief Returns a Vector rotated counterclockwise 90 deg.
   */
  Vector rotCounterClock() const;
};