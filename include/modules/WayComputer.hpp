#pragma once

#include <as_msgs/Tracklimits.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <fstream>
#include <queue>

#include "modules/Visualization.hpp"
#include "structures/Trace.hpp"
#include "structures/Vector.hpp"
#include "structures/Way.hpp"
#include "utils/KDTree.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

class WayComputer {
 private:
  const Params::WayComputer params_;
  Way way_;
  Way lastWay_;
  bool isLoopClosed_ = false;

  bool localTfValid_ = false;
  Eigen::Affine3d localTf_;

  using HeurInd = std::pair<float, size_t>;
  void filterTriangulation(TriangleSet &triangulation) const;

  void filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const;

  float getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir) const;

  void findNextEdges(std::vector<HeurInd> &nextEdges,
                     const Trace *actTrace,
                     const KDTree &midpointsKDT,
                     const std::vector<Edge> &edges) const;

  /**
   * @brief Performs the search in order to obtain the most probable way.
   * 
   * @param edges 
   */
  void computeWay(const std::vector<Edge> &edges);

 public:
  WayComputer(const Params::WayComputer &params);

  void poseCallback(const nav_msgs::Odometry::ConstPtr &data);

  void update(TriangleSet &triangulation);

  const bool &isLoopClosed() const;

  void writeWayToFile(const std::string &file_path) const;

  std::vector<Point> getPath() const;

  Tracklimits getTracklimits() const;

  as_msgs::PathLimits getPathLimits() const;
};