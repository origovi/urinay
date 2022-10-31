/**
 * @file Triangle.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Triangle class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "structures/Circle.hpp"
#include "structures/Edge.hpp"
#include "structures/Node.hpp"

/**
 * @brief Represents a triangle, includes all elements to ease the Delaunay
 * triangulation calculation.
 */
class Triangle {
 private:
  /**
   * @brief Circumcircle of the Triangle.
   */
  const Circle circumCircle_;

  /**
   * @brief Triangle's unique hash. The hash is calculated using the Node(s)' ids.
   */
  const size_t hash_;

  /**
   * @brief Returns the hash that would have a Triangle having \a n0, \a n1
   * and \a n2 as its Node(s).
   * 
   * @param[in] n0 
   * @param[in] n1 
   * @param[in] n2 
   */
  static size_t computeHash(const Node &n0, const Node &n1, const Node &n2);
  friend class std::hash<Triangle>;

 public:
  /**
   * @brief The 3 Node(s) defining a Triangle.
   */
  const std::array<Node, 3> nodes;

  /**
   * @brief The Edge(s) defined by the 3 Node(s).
   */
  const std::array<Edge, 3> edges;

  /**
   * @brief Construct a new Triangle object from 3 Node(s).
   * 
   * @param[in] n0 
   * @param[in] n1 
   * @param[in] n2 
   */
  Triangle(const Node &n0, const Node &n1, const Node &n2);
  
  /**
   * @brief Construct a new Triangle object from an Edge and a Node.
   * 
   * @param[in] e 
   * @param[in] n 
   */
  Triangle(const Edge &e, const Node &n);

  /**
 * @brief Comparison operator, two triangles will be equal when the Node(s)
 * are the same (the ids are checked).
 * 
 * @param[in] t 
 */
  bool operator==(const Triangle &t) const;

  /**
   * @brief Negation of the comparison operator.
   * 
   * @param[in] t 
   */
  bool operator!=(const Triangle &t) const;

  /**
   * @brief Checks if the Triangle contains the Node \a n.
   * 
   * @param[in] n 
   */
  bool containsNode(const Node &n) const;

  /**
   * @brief Checks if the Edge \a e belongs to the Triangle.
   * 
   * @param[in] e 
   */
  bool containsEdge(const Edge &e) const;

  /**
   * @brief Checks if a Node is inside the Triangle's circumcircle.
   * 
   * @param[in] n 
   */
  bool circleContainsNode(const Node &n) const;

  /**
   * @brief Checks if any Node of the Triangle belongs to the supertriangle.
   */
  bool anyNodeInSuperTriangle() const;

  /**
   * @brief Returns an array with the 3 angles of the Triangle.
   */
  std::array<double, 3> angles() const;

  /**
   * @brief Returns the circumcenter in local coordinates.
   */
  const Point &circumCenter() const;

  /**
   * @brief Returns the circumcenter in global coordinates.
   */
  const Point &circumCenterGlobal() const;

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] t 
   */
  friend std::ostream &operator<<(std::ostream &os, const Triangle &t);
};

template <>
struct std::hash<Triangle> {
  size_t operator()(const Triangle &t) const {
    return t.hash_;
  }
};