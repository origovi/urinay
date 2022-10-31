/**
 * @file DelaunayTri.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the DelaunayTri class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */


#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include "utils/definitions.hpp"
#include "structures/Triangle.hpp"

/**
 * @brief Static class that computes the Delaunay triangulation set
 * given a set of Node(s) (Cones).
 */
class DelaunayTri {
 private:
  /**
   * @brief Builds and returns a Triangle such that all Node(s) of \a nodes
   * are positioned inside.
   * 
   * @param[in] nodes 
   */
  static Triangle superTriangle(const std::vector<Node> &nodes);

 public:
  /**
   * @brief Computes the Delaunay triangulation set using an implementation
   * of the Bowyer-Watson algorithm to find the Delaunay triangulation given
   * a set of points. O(nlogn).
   * 
   * @param nodes 
   */
  static TriangleSet compute(const std::vector<Node> &nodes);
};