#pragma once

#include <as_msgs/Tracklimits.h>
#include <ros/ros.h>

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
  Way historic_;

  using HeurInd = std::pair<float, size_t>;
  void filterTriangulation(TriangleSet &triangulation) const;

  void filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const;

  float getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir) const;

  void findNextEdges(std::vector<HeurInd> &nextEdges,
                     const Edge *actEdge,
                     const Vector &dir,
                     const KDTree &midpointsKDT,
                     const std::vector<const Edge *> &edges) const;

  /**
   * @brief Performs the search in order to obtain the most probable way.
   * 
   * @param way 
   * @param edges 
   */
  void computeWay(Way &way, const std::vector<const Edge *> &edges) const;

 public:
  WayComputer(const Params::WayComputer &params);

  void update(TriangleSet &triangulation, const Visualization &vis);

  const Way &way() const;
};