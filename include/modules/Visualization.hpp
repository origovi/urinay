/**
 * @file Visualization.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Visualization class.
 * @version 1.0
 * @date 2022-09-07
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>

#include "structures/Triangle.hpp"
#include "modules/DelaunayTri.hpp"

class Visualization {
 private:
  ros::Publisher trianglesPub;
  ros::NodeHandle *const nh;

 public:
  Visualization(ros::NodeHandle *const nh, const std::string &triangulation_topic);
  void triangulation(const TriangleSet &triSet) const;
};

#endif  // VISUALIZATION_HPP