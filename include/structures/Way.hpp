#pragma once

#include <as_msgs/PathLimits.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <list>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

class Way {
 private:
  static Params::WayComputer::Way params_;

  std::list<Edge> path_;

 public:
  static void init(const Params::WayComputer::Way &params);

  bool empty() const;

  size_t size() const;

  const Edge &back() const;

  const Edge &beforeBack() const;

  const Edge &front() const;

  void updateLocal(const Eigen::Affine3d &tf);

  void addEdge(const Edge &edge);

  void trimByLocal();

  bool closesLoopWith(const Edge &e, const Point *lastPosInTrace = nullptr) const;

  void restructureClosure();

  bool closesLoop() const;

  bool containsEdge(const Edge &e) const;

  Way &operator=(const Way &way);

  bool operator==(const Way &way) const;

  bool operator!=(const Way &way) const;

  std::vector<Point> getPath() const;

  Tracklimits getTracklimits() const;

  friend std::ostream &operator<<(std::ostream &os, const Way &way);
};