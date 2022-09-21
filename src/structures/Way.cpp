#include "structures/Way.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

bool Way::empty() const {
  return this->path_.empty();
}

size_t Way::size() const {
  return this->path_.size();
}

const Edge &Way::back() const {
  return this->path_.back();
}

const Edge &Way::front() const {
  return this->path_.front();
}

void Way::addEdge(const Edge &edge) {
  this->path_.push_back(edge);
}

void Way::mergeWith(Way &way) {
  if (way.empty()) return;
  if (this->empty()) {
    this->path_.splice(this->path_.end(), way.path_);
    return;
  }

  // Update loop closing attribute
  this->canCloseLoopWith_ = this->canCloseLoopWith_ or Point::distSq(this->front().midPointGlobal(), way.front().midPointGlobal()) > this->MIN_DIST_PATHS_FRONT * this->MIN_DIST_PATHS_FRONT;

  double distSqWithFirst = Point::distSq(this->path_.front().midPointGlobal(), way.front().midPointGlobal());
  double smallestDWF = distSqWithFirst;
  auto smallestDWFIt = this->path_.begin();

  // Find closest element to way's first in this
  for (auto it = this->path_.begin(); it != this->path_.end(); it++) {
    distSqWithFirst = Point::distSq(it->midPointGlobal(), way.front().midPointGlobal());
    if (distSqWithFirst <= smallestDWF) {
      smallestDWF = distSqWithFirst;
      smallestDWFIt = it;
    }
  }

  // Erase all elements after closest element
  this->path_.erase(smallestDWFIt, this->path_.end());

  // Append new path into this
  this->path_.splice(this->path_.end(), way.path_);

  // Restructure closure
  if (this->closesLoop()) {
    // this->restructureClosure();
  }
}

bool Way::closesLoop() const {
  return this->size() >= this->MIN_SIZE_FOR_LOOP and Point::distSq(this->front().midPointGlobal(), this->back().midPointGlobal()) < this->MIN_DIST_LOOP_CLOSURE * this->MIN_DIST_LOOP_CLOSURE;
}

bool Way::closesLoopWith(const Way &way) const {
  if (this->empty() or way.empty() or (this->size() < this->MIN_SIZE_FOR_LOOP and way.size() < this->MIN_SIZE_FOR_LOOP)) return false;
  return canCloseLoopWith_ and Point::distSq(this->front().midPointGlobal(), way.back().midPointGlobal()) < this->MIN_DIST_LOOP_CLOSURE * this->MIN_DIST_LOOP_CLOSURE;
}

void Way::restructureClosure() {
  if (this->front() == this->back()) return;

  // Assume last Edge is the one that closes the loop
  double distClosestWithLast = Point::distSq(this->front().midPointGlobal(), this->back().midPointGlobal());
  auto closestWithLastIt = this->path_.begin();

  for (auto it = this->path_.begin(); it != std::prev(this->path_.end(), 5); it++) {  // The 5 is for safety
    double distWithLast = Point::distSq(this->back().midPointGlobal(), it->midPointGlobal());
    if (distWithLast <= distClosestWithLast) {
      distClosestWithLast = distWithLast;
      closestWithLastIt = it;
    }
  }

  // We remove all edges that would cause our "loop" not to be a loop
  // (all that are before the edge that closes the loop)
  this->path_.erase(this->path_.begin(), closestWithLastIt);

  // Finally, we make sure that the first edge is the same as the last
  // (to surely create a loop)
  if (this->front() != this->back()) this->path_.push_front(this->back());
}

bool Way::containsEdge(const Edge &e) const {
  for (const Edge &edge : this->path_) {
    if (e == edge) return true;
  }
  return false;
}

std::vector<Point> Way::getPath() const {
  std::vector<Point> res;
  res.reserve(this->path_.size());
  for (const Edge &e : this->path_) {
    res.push_back(e.midPointGlobal());
  }
  return res;
}

Tracklimits Way::getTracklimits() const {
  Tracklimits res;
  res.first.reserve(this->size());
  res.second.reserve(this->size());
  Point pAnt(0, 0);  // This only works in a global

  const Node *left;
  const Node *right;

  for (const Edge &e : this->path_) {
    Vector pAntPAct(pAnt, e.midPointGlobal());

    if (Vector::pointBehind(e.n0.pointGlobal(), e.midPointGlobal(), pAntPAct.rotClock())) {
      left = &e.n1;
      right = &e.n0;
    } else {
      left = &e.n0;
      right = &e.n1;
    }

    if (*left != res.first.back())
      res.first.push_back(*left);
    if (*right != res.second.back())
      res.second.push_back(*right);

    pAnt = e.midPointGlobal();
  }
  return res;
}

as_msgs::PathLimits Way::getPathLimits() const {
  as_msgs::PathLimits res;
  res.stamp = ros::Time::now();
  res.replan = true;

  // Fill path
  std::vector<Point> path = this->getPath();
  res.path.reserve(path.size());
  for (const Point &p : path) {
    res.path.push_back(p.gmPoint());
  }

  // Fill Tracklimits
  Tracklimits tracklimits = this->getTracklimits();
  res.tracklimits.left.reserve(tracklimits.first.size());
  for (const Node &n : tracklimits.first) {
    res.tracklimits.left.push_back(n.cone());
  }
  for (const Node &n : tracklimits.second) {
    res.tracklimits.right.push_back(n.cone());
  }
  res.tracklimits.stamp = res.stamp;
  res.tracklimits.replan = res.replan;
  return res;
}