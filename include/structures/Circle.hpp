/**
 * @file Circle.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Circle class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cmath>
#include <iostream>

#include "structures/Node.hpp"

/**
 * @brief Represents a circle in 2D coordinates.
 */
class Circle {
 private:
  /**
   * @brief The center of the Circle both in local and global coordinates.
   */
  Point center_, centerGlobal_;

  /**
   * @brief The radius of the Circle to the power of two.
   */
  double radSq_;

 public:
  /**
   * @brief Construct a new Circle object.
   * \a n0, \a n1, \a n2 are 3 Node(s) belonging to the Circle's circumference.
   * 
   * @param[in] n0
   * @param[in] n1 
   * @param[in] n2 
   */
  Circle(const Node &n0, const Node &n1, const Node &n2);

  /**
   * @brief Checks if Node \a n is inside the Circle's circumference.
   * 
   * @param n 
   */
  bool containsNode(const Node &n) const;

  /**
   * @brief Returns the Circle's center in local coordinates.
   */
  const Point &center() const;

  /**
   * @brief Returns the Circle's center in global coordinates.
   */
  const Point &centerGlobal() const;
};