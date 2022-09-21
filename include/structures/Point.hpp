/**
 * @file Point.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Point class.
 * @version 1.0
 * @date 2022-09-16
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
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
   * @param p1 
   * @param p2 
   * @return double 
   */
  static inline double distSq(const Point &p1, const Point &p2 = Point()) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
  }

  /**
   * @brief Performs the Euclidean distance between two Point(s).
   * 
   * @param p1 
   * @param p2 
   * @return double 
   */
  static inline double dist(const Point &p1, const Point &p2 = Point()) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }

  /**
   * @brief Prints a Point.
   * 
   * @param os 
   * @param p 
   * @return std::ostream& 
   */
  friend std::ostream &operator<<(std::ostream &os, const Point &p);

  /**
   * @brief Returns the tranformed Point with an Eigen::Affine3d.
   * 
   * @param tf 
   * @return Point 
   */
  Point transformed(const Eigen::Affine3d &tf) const;

  /**
   * @brief Converts the point to a geometry_msgs::Point.
   * 
   * @return geometry_msgs::Point 
   */
  geometry_msgs::Point gmPoint() const;

  /**
   * @brief Returns the coordinate of the position specified by \a ind.
   * 
   * @param ind 
   * @return const double& 
   */
  const double &at(const size_t &ind) const;

  /**
   * @brief Returns 2.
   * 
   * @return size_t 
   */
  size_t size() const;
};