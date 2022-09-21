#pragma once

#include <as_msgs/PathLimits.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <list>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/definitions.hpp"

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

  std::list<Edge> path_;

 public:
  bool empty() const;

  size_t size() const;

  const Edge &back() const;

  const Edge &beforeBack() const;

  const Edge &front() const;

  void updateLocal(const Eigen::Affine3d &tf);

  void addEdge(const Edge &edge);

  void trimByLocal();

  bool closesLoopWith(const Edge &e) const;

  void restructureClosure();

  bool closesLoop() const;

  bool containsEdge(const Edge &e) const;

  std::vector<Point> getPath() const;

  Tracklimits getTracklimits() const;

  as_msgs::PathLimits getPathLimits() const;

  friend std::ostream &operator<<(std::ostream &os, const Way &way);
};