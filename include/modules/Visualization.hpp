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

#include "structures/Way.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

class Visualization {
 private:
  ros::Publisher trianglesPub, midpointsPub, wayPub;
  ros::NodeHandle *const nh;
  const Params::Visualization params_;

 public:
  Visualization(ros::NodeHandle *const nh, const Params::Visualization &params);
  void visualize(const TriangleSet &triSet) const;
  void visualize(const EdgeSet &edgeSet) const;
  void visualize(const Way &way) const;
};

#endif  // VISUALIZATION_HPP