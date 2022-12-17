/**
 * @file Point.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Point class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <geometry_msgs/Point.h>

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>

/**
 * @brief Represents a Point in 2D and contains useful tools to perform
 * operations with them, also to convert them to ROS msgs.
 */
class Point {
 public:
  /* -------------------------- Public Constructors ------------------------- */

  Point();
  Point(const double &x, const double &y);
  template <typename T>
  Point(const T &point);

  /* --------------------------- Public Attributes -------------------------- */

  double x, y;

  /* ---------------------------- Public Methods ---------------------------- */

  Point operator+(const Point &p) const;
  Point operator-(const Point &p) const;

  template <typename T>
  Point operator*(const T &num) const;

  template <typename T>
  Point operator/(const T &num) const;

  Point &operator+=(const Point &p);
  Point &operator-=(const Point &p);

  template <typename T>
  Point &operator*=(const T &num);

  template <typename T>
  Point &operator/=(const T &num);

  /**
   * @brief Performs the squared Euclidean distance between two Point(s).
   * 
   * @param[in] p1 
   * @param[in] p2 
   */
  static inline double distSq(const Point &p1, const Point &p2 = Point()) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
  }

  /**
   * @brief Performs the Euclidean distance between two Point(s).
   * 
   * @param[in] p1 
   * @param[in] p2 
   */
  static inline double dist(const Point &p1, const Point &p2 = Point()) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }

  /**
   * @brief Checks if the orientation of ordered triplet (A, B, C) is
   * counterclockwise.
   * 
   * @param[in] A 
   * @param[in] B 
   * @param[in] C 
   */
  static inline bool ccw(const Point &A, const Point &B, const Point &C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  }

  /**
   * @brief Prints a Point.
   * 
   * @param[in,out] os 
   * @param[in] p 
   */
  friend std::ostream &operator<<(std::ostream &os, const Point &p);

  /**
   * @brief Returns the tranformed Point with an Eigen::Affine3d.
   * 
   * @param[in] tf 
   */
  Point transformed(const Eigen::Affine3d &tf) const;

  /**
   * @brief Converts the point to a geometry_msgs::Point.
   */
  geometry_msgs::Point gmPoint() const;

  /**
   * @brief Returns the coordinate of the position specified by \a ind.
   * 
   * @param[in] ind 
   */
  const double &at(const size_t &ind) const;

  /**
   * @brief Returns 2.
   */
  size_t size() const;
};