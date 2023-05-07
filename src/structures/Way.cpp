/**
 * @file Way.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Way class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Way.hpp"

/* ----------------------------- Private Methods ---------------------------- */

Params::WayComputer::Way Way::params_;

void Way::updateClosestToCarElem() {
  auto smallestDWFIt = this->path_.cbegin();
  if (!this->path_.empty()) {
    double smallestDWF = Point::distSq(this->path_.front().midPoint());

    // Find closest element to way's first in this
    for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++) {
      double distSqWithFirst = Point::distSq(it->midPoint());
      if (distSqWithFirst <= smallestDWF) {
        smallestDWF = distSqWithFirst;
        smallestDWFIt = it;
      }
    }
  }
  this->closestToCarElem_ = smallestDWFIt;
}

bool Way::segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D) {
  return Point::ccw(A, C, D) != Point::ccw(B, C, D) and Point::ccw(A, B, C) != Point::ccw(A, B, D);
}

/* ----------------------------- Public Methods ----------------------------- */

void Way::init(const Params::WayComputer::Way &params) {
  params_ = params;
}

Way::Way()
    : avgEdgeLen_(0.0) {
  closestToCarElem_ = this->path_.cend();
  sizeToCar_ = 0;
}

bool Way::empty() const {
  return this->path_.empty();
}

size_t Way::size() const {
  return this->path_.size();
}

const Edge &Way::back() const {
  return this->path_.back();
}

const Edge &Way::beforeBack() const {
  ROS_ASSERT(this->size() >= 2);
  return *(++this->path_.rbegin());
}

const Edge &Way::front() const {
  return this->path_.front();
}

void Way::updateLocal(const Eigen::Affine3d &tf) {
  for (Edge &e : this->path_) {
    e.updateLocal(tf);
  }
  // Update closest
  this->updateClosestToCarElem();
}

void Way::addEdge(const Edge &edge) {
  this->path_.push_back(edge);
  if (this->path_.size() == 1) closestToCarElem_ = this->path_.cbegin();
  this->avgEdgeLen_ += (edge.len - this->avgEdgeLen_) / this->size();
}

void Way::trimByLocal() {
  if (this->size() < 2) return;

  auto closestToCarElem = this->closestToCarElem_;

  if (closestToCarElem != std::prev(this->path_.cend())) {
    // Erase all elements after closest element (included)
    this->path_.erase(++closestToCarElem, this->path_.cend());
  }

  // Recalculate the mean edge length attribute
  size_t t = 1;
  this->avgEdgeLen_ = 0.0;
  for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++) {
    this->avgEdgeLen_ += (it->len - this->avgEdgeLen_) / t;
    t++;
  }

  // Recalculate sizeToCar_ attribute
  this->sizeToCar_ = this->size();
}

bool Way::closesLoop() const {
  return this->size() >= MIN_LOOP_SIZE and Point::distSq(this->front().midPoint(), this->back().midPoint()) <= params_.max_dist_loop_closure * params_.max_dist_loop_closure;
}

bool Way::closesLoopWith(const Edge &e, const Point *lastPosInTrace) const {
  Point actPos = lastPosInTrace ? *lastPosInTrace : this->back().midPoint();
  return
      // Must have a minimum size
      this->size() + 1 >= MIN_LOOP_SIZE and
      // Distance conditions are met
      Point::distSq(this->front().midPoint(), e.midPoint()) <= params_.max_dist_loop_closure * params_.max_dist_loop_closure and
      // Check closure angle with first point
      abs(Vector(this->front().midPoint(), (++this->path_.begin())->midPoint()).angleWith(Vector(actPos, e.midPoint()))) <= params_.max_angle_diff_loop_closure;
}

Way Way::restructureClosure() {
  Way res = *this;
  if (res.front() != res.back()) {
    // Assume last Edge is the one that closes the loop
    double distClosestWithLast = Point::distSq(res.front().midPoint(), res.back().midPoint());
    auto closestWithLastIt = res.path_.begin();

    for (auto it = res.path_.begin(); it != std::prev(res.path_.end(), 5); it++) {  // The 5 is for safety
      double distWithLast = Point::distSq(res.back().midPoint(), it->midPoint());
      if (distWithLast <= distClosestWithLast) {
        distClosestWithLast = distWithLast;
        closestWithLastIt = it;
      }
    }

    // We remove all edges that would cause our "loop" not to be a loop
    // (all that are before the edge that closes the loop)
    res.path_.erase(res.path_.begin(), closestWithLastIt);
  }
  // Note that here only the ids are equal, not necessarily the midpoint
  // this is why the midpoint needs to be updated.
  if (res.front() == res.back()) res.path_.pop_back();
  res.path_.push_back(res.front());

  return res;
}

bool Way::intersectsWith(const Edge &e) const {
  if (this->size() <= 2) return false;
  Point s1p1, s1p2;
  const Point s2p1 = this->back().midPoint();
  const Point s2p2 = e.midPoint();
  auto it1 = std::next(this->path_.crbegin());
  auto it2 = std::next(it1);
  while (it2 != this->path_.crend()) {
    s1p1 = it1->midPoint();
    s1p2 = it2->midPoint();
    if (this->segmentsIntersect(s1p1, s1p2, s2p1, s2p2)) return true;
    it1++;
    it2++;
  }
  return false;
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
  Point pAnt = this->empty() ? Point(0, 0) : this->front().midPointGlobal() - Point(5, 0);  // This only works in a global, the 5 is arbitrary

  const Node *left;
  const Node *right;

  for (const Edge &e : this->path_) {
    Vector pAntPAct(pAnt, e.midPointGlobal());

    // Check the side of both Nodes of the Edge
    if (Vector::pointBehind(e.n0.pointGlobal(), e.midPointGlobal(), pAntPAct.rotClock())) {
      left = &e.n0;
      right = &e.n1;
    } else {
      left = &e.n1;
      right = &e.n0;
    }

    // Only append those Nodes that have not been appended before
    if (*left != res.first.back())
      res.first.push_back(*left);
    if (*right != res.second.back())
      res.second.push_back(*right);

    pAnt = e.midPointGlobal();
  }
  return res;
}

Way &Way::operator=(const Way &way) {
  this->path_ = std::list<Edge>(way.path_);
  this->avgEdgeLen_ = way.avgEdgeLen_;
  this->sizeToCar_ = way.sizeToCar_;
  this->updateClosestToCarElem();  // A copy of the attribute would be unsafe
  return *this;
}

bool Way::operator==(const Way &way) const {
  if (this->size() != way.size()) return false;
  auto thisIt = this->path_.cbegin();
  auto paramIt = way.path_.cbegin();

  while (thisIt != this->path_.cend()) {
    if (*thisIt != *paramIt) return false;
    thisIt++;
    paramIt++;
  }

  return true;
}

bool Way::operator!=(const Way &way) const {
  return not(*this == way);
}

bool Way::quinEhLobjetiuDeLaSevaDiresio(const Way &way) const {
  // Replan every time the emptiness changes
  if (this->empty() != way.empty()) return true;
  // Find the common starting point (replan if not found)
  auto wayIt = way.closestToCarElem_;
  auto thisIt = this->closestToCarElem_;

  while (wayIt != way.path_.cend()) {
    if (*wayIt == *thisIt) {
      // Common starting point found!
      break;
    }
    wayIt++;
  }

  // Case no starting point has been found
  if (*wayIt != *thisIt) return true;

  // Compare one by one the vital_num_midpoints
  int midpoint_num = 0;
  while (wayIt != way.path_.cend() and thisIt != this->path_.cend()) {
    if (midpoint_num >= this->params_.vital_num_midpoints) return false;
    if (*wayIt != *thisIt) return true;
    midpoint_num++;
    wayIt++;
    thisIt++;
  }

  // Case where we reach the end of one of the two Way(s), we could check if
  // one is longer than the other (within these vital_num_midpoints) and return
  // accordingly, but this case is better covered by the planner itself (it will
  // replan when it runs out of midpoints).
  return false;
}

const double &Way::getAvgEdgeLen() const {
  return this->avgEdgeLen_;
}

uint32_t Way::sizeAheadOfCar() const {
  return this->path_.size() - this->sizeToCar_;
}


std::ostream &operator<<(std::ostream &os, const Way &way) {
  Tracklimits tracklimits = way.getTracklimits();
  for (const Node &n : tracklimits.first) {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 0 << ' ' << n.id << std::endl;
  }
  for (const Node &n : tracklimits.second) {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 1 << ' ' << n.id << std::endl;
  }
  return os;
}