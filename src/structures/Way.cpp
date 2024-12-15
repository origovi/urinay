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

/* ----------------------------- Public Methods ----------------------------- */

void Way::init(const Params::WayComputer::Way &params) {
  params_ = params;
}

Way::Way() {
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
  this->edgesInPath_.insert(this->path_.back());
  if (this->path_.size() == 1) closestToCarElem_ = this->path_.cbegin();
}

void Way::trimByLocal() {
  if (this->size() < 2) return;

  auto closestToCarElem = this->closestToCarElem_;

  if (closestToCarElem != std::prev(this->path_.cend())) {
    // Update this->edgesInPath_ param
    for (auto it = std::next(closestToCarElem); it != std::prev(this->path_.cend()); it++) {
      this->edgesInPath_.erase(*it);
    }
    // Avoid deleting an edge in this->edgesInPath_ that is actually in this->path_
    if (this->front() != this->back()) this->edgesInPath_.erase(*std::prev(this->path_.cend()));

    // Erase all elements after closest element (included)
    this->path_.erase(++closestToCarElem, this->path_.cend());
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

Way Way::restructureClosure() const {
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
  // Here last midpoint (now the closest to the first one) will be replaced
  // by the first one. Making sure that they are EXACTLY the same (id and value).
  if (res.front() == res.back() or
      Vector::pointBehind(res.back().midPointGlobal(), res.front().midPointGlobal(), Vector(res.beforeBack().midPointGlobal(), res.back().midPointGlobal())) or
      Point::dist(res.back().midPointGlobal(), res.front().midPointGlobal()) < SAME_MIDPOINT_DIST_THRESHOLD) {
    res.path_.pop_back();
  }
  res.path_.push_back(res.front());

  return res;
}

bool Way::intersectsWith(const Edge &e, const Trace* const actTrace, const std::vector<Edge> &edges) const {
  size_t trace_size = actTrace ? actTrace->size() : 0;
  if (this->size() + trace_size < 3) return false;
  Point s1p1, s1p2;
  const Point s2p1 = e.midPoint();
  const Point s2p2 = actTrace ? edges[actTrace->edgeInd()].midPoint() : this->back().midPoint();

  // Check if intersects with Trace
  if (actTrace) {
    Trace aux = actTrace->before();
    if (!aux.empty())
      s1p1 = edges[aux.edgeInd()].midPoint();
    while (aux.size() >= 2) {
      aux = aux.before();
      s1p2 = edges[aux.edgeInd()].midPoint();
      if (this->segmentsIntersect(s1p1, s1p2, s2p1, s2p2)) return true;
      s1p1 = s1p2;
    }
  }

  if (this->empty()) return false;
  
  // Check if intersects with Way
  std::list<Edge>::const_reverse_iterator it;
  if (actTrace) {
    it = this->path_.crbegin();
    if (trace_size < 2) {
      s1p1 = it->midPoint();
      it++;
    }
  }
  else {
    it = std::next(this->path_.crbegin());
    s1p1 = it->midPoint();
    it++;
  }
  while (it != this->path_.crend()) {
    s1p2 = it->midPoint();
    if (this->segmentsIntersect(s1p1, s1p2, s2p1, s2p2)) return true;
    s1p1 = s1p2;
    it++;
  }
  return false;
}

bool Way::containsEdge(const Edge &e) const {
  return this->edgesInPath_.find(e) != this->edgesInPath_.cend();
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
  Point pAnt = this->empty() ? Point(0, 0) : Point(-50, this->front().midPointGlobal().y);  // This only works in a global, the -50 is arbitrary

  const Node *left, *firstLeft;
  const Node *right, *firstRight;

  size_t edgeInd = 0;
  for (const Edge &e : this->path_) {
    // if (this->closesLoop() and edgeInd == this->size() - 1) break;  // Last Edge is (can be) same as first => nothing to append
    Vector pAntPAct(pAnt, e.midPointGlobal());

    // Check the side of both Nodes of the Edge
    if (Vector::pointBehind(e.n0.pointGlobal(), e.midPointGlobal(), pAntPAct.rotClock())) {
      left = &e.n0;
      right = &e.n1;
    } else {
      left = &e.n1;
      right = &e.n0;
    }

    // Save first Nodes
    if (edgeInd == 0) {
      firstLeft = left;
      firstRight = right;
    }

    // Only append those Nodes that:
    // - have not been appended before
    // - have a position behind the last one
    // - [only for the 3 lasts (if loop closure)] have a position in front of the first one
    if (res.first.empty() or
        (*left != res.first.back() and
         !Vector::pointBehind(left->pointGlobal(), res.first.back().pointGlobal(), pAntPAct) and
         (!this->closesLoop() or edgeInd < this->size() - 3 or !Vector::pointBehind(res.first.front().pointGlobal(), left->pointGlobal(), pAntPAct)))) {
      if (*left == *firstLeft) {
        res.first.push_back(*firstLeft);
      }
      else res.first.push_back(*left);
    }
    if (res.second.empty() or
        (*right != res.second.back() and
         !Vector::pointBehind(right->pointGlobal(), res.second.back().pointGlobal(), pAntPAct) and
         (!this->closesLoop() or edgeInd < this->size() - 3 or !Vector::pointBehind(res.second.front().pointGlobal(), right->pointGlobal(), pAntPAct)))) {
      if (*right == *firstRight) {
        res.second.push_back(*firstRight);
      }
      else res.second.push_back(*right);
    }

    pAnt = e.midPointGlobal();
    edgeInd++;
  }
  return res;
}

Way &Way::operator=(const Way &way) {
  this->path_ = std::list<Edge>(way.path_);
  this->edgesInPath_ = way.edgesInPath_;
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

bool Way::vitalMidpointsChanged(const Way &way) const {
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