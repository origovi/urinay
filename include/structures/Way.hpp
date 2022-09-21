#pragma once

#include <list>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/definitions.hpp"

#include <as_msgs/PathLimits.h>

class Way {
 private:
  // Min loop size in order to make it eligible to be closed.
  const int MIN_SIZE_FOR_LOOP = 25;
  
  // At some point there MUST be a minimum distance of X meters between
  // this->front().midpointGlobal() and way.front().midpointGlobal(), to avoid
  // false detections of loop closure at the beginning.
  const double MIN_DIST_PATHS_FRONT = 5.0;

  // Min distance between path front and back to detect it as a loop.
  const double MIN_DIST_LOOP_CLOSURE = 2.0;

  bool canCloseLoopWith_ = false;
  std::list<Edge> path_;

  void restructureClosure();

 public:
  bool empty() const;

  size_t size() const;

  const Edge &back() const;

  const Edge &front() const;

  void addEdge(const Edge &edge);

  void mergeWith(Way &way);

  bool closesLoopWith(const Way &way) const;

  bool closesLoop() const;

  bool containsEdge(const Edge &e) const;

  std::vector<Point> getPath() const;

  Tracklimits getTracklimits() const;

  as_msgs::PathLimits getPathLimits() const;
};