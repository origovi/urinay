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
}

bool Way::closesLoop() const {
  return this->size() >= this->MIN_SIZE_FOR_LOOP and Point::distSq(this->front().midPointGlobal(), this->back().midPointGlobal()) < 0.5 * 0.5;
}

bool Way::closesLoopWith(const Way &way) const {
  std::cout << this->size() << ' ' << way.size() << std::endl;
  if (this->empty() or way.empty() or (this->size() < this->MIN_SIZE_FOR_LOOP and way.size() < this->MIN_SIZE_FOR_LOOP)) return false;
  return canCloseLoopWith_ and Point::distSq(this->front().midPointGlobal(), way.back().midPointGlobal()) < this->MIN_DIST_LOOP_CLOSURE * this->MIN_DIST_LOOP_CLOSURE;
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