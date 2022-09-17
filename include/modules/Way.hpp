#pragma once

#include <as_msgs/Tracklimits.h>

#include <queue>

#include "modules/Visualization.hpp"
#include "structures/Trace.hpp"
#include "utils/definitions.hpp"
#include "utils/KDTree.hpp"
#include "utils/Params.hpp"

class Way {
 private:
  const Params::Way params_;
  Path historic_;

  void filterTriangulation(TriangleSet &triangulation) const;

  void filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const;

  void findNextPossibleEdges(std::vector<std::pair<size_t, float>> &nextPossibleEdges, const Point &point, const Point &dir, const KDTree &midpointsKDT) const;
  
  /**
   * @brief Performs the search in order to obtain the most probable path.
   * 
   * @param path 
   * @param edges 
   */
  void computePath(Path &path, const std::vector<const Edge *> &edges) const;

  /**
   * @brief Updates the historic path \a historic_ with the new path.
   * 
   * @param path 
   */
  void mergeToHistoric(const Path &path);

 public:
  Way(const Params::Way &params);

  void update(TriangleSet &triangulation, const Visualization &vis);
};