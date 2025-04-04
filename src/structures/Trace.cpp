/**
 * @file Trace.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Trace class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Trace.hpp"

/* ----------------------------- Private Methods ---------------------------- */

Params::WayComputer::Way Trace::params_;

Trace::Connection::Connection(const Edge &edge, const double &heur, const bool &loopClosed, std::shared_ptr<Connection> before)
    : edge(edge), before(before), heur(heur),
      sumHeur(before ? before->sumHeur + heur : heur), size(before ? (before->size + 1) : 1), len(before ? (before->len + Point::dist(edge.midPoint(), before->edge.midPoint())) : 0.0),
      avgTrackWidth(before ? before->avgTrackWidth + ((edge.trackWidth(Vector(before->edge.midPoint(), edge.midPoint())) - before->avgTrackWidth) / (before->size + 1)) : edge.trackWidth(Vector(1, 0))),
      loopClosed(loopClosed),
      connectionsSinceLoopClosed(before && before->connectionsSinceLoopClosed >= 1 ? before->connectionsSinceLoopClosed + 1 : (loopClosed ? 1 : 0)) {}

bool Trace::Connection::containsEdge(const Edge &_edge) const {
  return edge == _edge || (before ? before->containsEdge(_edge) : false);
}

Trace::Trace(const std::shared_ptr<Connection> &p, const size_t &sizeToCar)
    : p(p), sizeToCar_(sizeToCar) {}

/* ----------------------------- Public Methods ----------------------------- */

void Trace::init(const Params::WayComputer::Way &params) {
  params_ = params;
}

Trace::Trace()
    : p(nullptr), sizeToCar_(0) {}

Trace::Trace(const Edge &edge, const double &heur, const Trace &previous) {
  this->sizeToCar_ = previous.sizeToCar_;
  bool loopClosed = previous.isLoopClosed() or previous.closesLoopWith(edge);
  this->p = std::make_shared<Connection>(edge, heur, loopClosed, previous.p);
}

void Trace::addEdge(const Edge &edge, const double &heur) {
  bool loopClosed = this->isLoopClosed() or this->closesLoopWith(edge);
  this->p = std::make_shared<Connection>(edge, heur, loopClosed, this->p);
}

bool Trace::empty() const {
  return not bool(this->p);
}

size_t Trace::size() const {
  if (empty())
    return 0;
  else
    return this->p->size;
}

double Trace::length() const {
  if (empty())
    return 0.0;
  else
    return this->p->len;
}

size_t Trace::sizeAheadOfCar() const { return this->size() - this->sizeToCar_; }

Trace Trace::before(const uint64_t &num) const {
  ROS_ASSERT(this->size() + 1 >= num);
  std::shared_ptr<Connection> aux = p;
  for (uint64_t i = 0; i < num; i++) {
    aux = aux->before;
  }
  return Trace(aux, std::min(aux ? aux->size : 0, this->sizeToCar_));
}

Trace Trace::first() const {
  ROS_ASSERT(not empty());
  std::shared_ptr<Connection> lastNotEmpty = p;
  while (lastNotEmpty->before != nullptr) {
    lastNotEmpty = lastNotEmpty->before;
  }
  return Trace(lastNotEmpty, 1);
}

Trace Trace::second() const {
  ROS_ASSERT(this->size() >= 2);
  std::shared_ptr<Connection> aux = this->p;
  while (aux->size > 2) {
    aux = aux->before;
  }
  return Trace(aux, std::min(aux->size, this->sizeToCar_));
}

const Edge &Trace::edge() const {
  ROS_ASSERT(not empty());
  return this->p->edge;
}

const Edge &Trace::beforeBack() const {
  ROS_ASSERT(this->size() >= 2);
  return this->p->before->edge;
}

const double &Trace::heur() const {
  ROS_ASSERT(not empty());
  return this->p->heur;
}

double Trace::avgTrackWidth() const {
  if (empty()) return 0.0;
  else return this->p->avgTrackWidth;
}

bool Trace::isLoopClosed() const {
  if (empty()) return false;
  return this->p->loopClosed;
}

double Trace::sumHeur() const {
  if (empty())
    return std::numeric_limits<double>::infinity();
  else
    return this->p->sumHeur;
}

bool Trace::containsEdge(const Edge &edge) const {
  if (empty())
    return false;
  else
    return this->p->containsEdge(edge);
}

bool Trace::closesLoopWith(const Edge &e) const {
  if (this->empty()) return false;

  Point actPos = this->edge().midPoint();
  Point firstPos = this->first().edge().midPoint();
  return
      // Must have a minimum size
      this->size() + 1 >= MIN_LOOP_SIZE and
      // Distance conditions are met
      Point::distSq(firstPos, e.midPoint()) <= params_.max_dist_loop_closure * params_.max_dist_loop_closure and
      // Check closure angle with first point
      abs(Vector(firstPos, this->second().edge().midPoint()).angleWith(Vector(actPos, e.midPoint()))) <= params_.max_angle_diff_loop_closure;
}

uint32_t Trace::connectionsSinceLoopClosed() const {
  if (empty())
    return 0;
  else
    return this->p->connectionsSinceLoopClosed;
}

void Trace::trimByLocal(const Eigen::Vector3d &carPosition) {
  if (this->size() < 2) return;

  bool loopClosed = this->isLoopClosed();

  double smallestDWF = Point::distSq(this->edge().midPoint());
  for (Trace aux = *this; !aux.empty(); aux = aux.before()) {
    double distSqWithFirst = Point::distSq(aux.edge().midPoint());
    if (distSqWithFirst < smallestDWF) {
      smallestDWF = distSqWithFirst;
      *this = aux;
      this->sizeToCar_ = this->size();
    }
  }
  // Case where the car is at the start line (recompute from empty)
  if (!loopClosed and this->size() == 1 and Point::distSq({carPosition.x(), carPosition.y()}) < CAR_HASNT_MOVED_START_RADIUS * CAR_HASNT_MOVED_START_RADIUS) {
    *this = Trace();
  }
}

Trace Trace::trimLoopClosure() const {
  Trace res = *this;
  while (!res.empty() and res.connectionsSinceLoopClosed() >= 2) {
    res = res.before();
  }
  return res;
}

Trace Trace::restructureClosure() const {
  Trace res = *this;
  if (res.size() < MIN_LOOP_SIZE) {
    ROS_WARN("[urinay] Attempting to restructure closure of a short Trace.");
    return res;
  }

  const Edge &front = res.first().edge();
  if (front != res.edge()) {
    // Assume last Edge is the one that closes the loop
    double distClosestWithLast = Point::distSq(front.midPoint(), res.edge().midPoint());
    Trace closestWithLast = res;

    for (Trace aux = res.before(5); !aux.empty(); aux = aux.before()) {  // The 5 is for safety and arbitrary
      double distWithLast = Point::distSq(res.edge().midPoint(), aux.edge().midPoint());
      if (distWithLast <= distClosestWithLast) {
        distClosestWithLast = distWithLast;
        closestWithLast = aux;
      }
    }

    // We remove all edges that would cause our "loop" not to be a loop
    // (all that are before the edge that closes the loop)
    closestWithLast.detach();
  }
  const Edge &firstEdge = res.first().edge();
  // Here last midpoint (now the closest to the first one) will be replaced
  // by the first one. Making sure that they are EXACTLY the same (id and value).
  if (firstEdge == res.edge() or
      Vector::pointBehind(res.edge().midPointGlobal(), firstEdge.midPointGlobal(), Vector(res.beforeBack().midPointGlobal(), res.edge().midPointGlobal())) or
      Point::dist(res.edge().midPointGlobal(), firstEdge.midPointGlobal()) < SAME_MIDPOINT_DIST_THRESHOLD) {
    res = res.before();
  }
  res.addEdge(firstEdge, 0.0);  // The heur value should not matter here

  return res;
}


bool Trace::intersectsWith(const Edge &e) const {
  if (this->size() < 3) return false;
  Point s1p1, s1p2;
  const Point s2p1 = e.midPoint();
  const Point s2p2 = this->edge().midPoint();

  // Check if intersects with Trace
  Trace aux = this->before();
  s1p1 = aux.edge().midPoint();
  while (aux.size() >= 2) {
    aux = aux.before();
    s1p2 = aux.edge().midPoint();
    if (Point::segmentsIntersect(s1p1, s1p2, s2p1, s2p2)) return true;
    s1p1 = s1p2;
  }
  return false;
}

void Trace::clear() {
  this->p = nullptr;
  this->sizeToCar_ = 0;
}

void Trace::detach() const {
  if (this->empty()) return;
  this->p->before.reset();
}

void Trace::updateLocal(const Eigen::Affine3d &tf) const {
  if (this->empty()) return;
  std::shared_ptr<Connection> aux = p;
  while (aux) {
    aux->edge.updateLocal(tf);
    aux = aux->before;
  }
}

bool Trace::operator<(const Trace &t) const {
  return this->size() > t.size() or (this->size() == t.size() and this->sumHeur() < t.sumHeur());
}

bool Trace::operator==(const Trace &t) const {
  if (this->size() != t.size()) return false;

  std::shared_ptr<Connection> p1 = this->p;
  std::shared_ptr<Connection> p2 = t.p;

  while (p1) {
    if (p1->edge != p2->edge) return false;
    p1 = p1->before;
    p2 = p2->before;
  }

  return true;
}

bool Trace::operator!=(const Trace &t) const {
  return not(*this == t);
}

bool Trace::vitalMidpointsChanged(const Trace &t) const {
  // Replan every time the emptiness changes
  if (this->empty() != t.empty()) return true;
  
  Trace this_aux = *this;
  Trace t_aux = t;

  // Find last vital midpoint of this
  this_aux = this_aux.before(std::max(int(this->sizeAheadOfCar()) - params_.vital_num_midpoints, 0));

  // Find last vital midpoint in t
  while (!t_aux.empty() and this_aux.edge() != t_aux.edge()) {
    t_aux = t_aux.before();
  }
  if (t_aux.empty() or this_aux.edge() != t_aux.edge())
    return true;

  t_aux = t_aux.before(); this_aux = this_aux.before();

  // Check one by one the vital midpoints so they are the same
  while (!this_aux.empty() and !t_aux.empty() and this_aux.size() != this_aux.sizeToCar_) {
    if (this_aux.edge() != t_aux.edge())
      return true;
    t_aux = t_aux.before(); this_aux = this_aux.before();
  }
  
  return false;
}

// Useful to print a Trace
std::ostream &operator<<(std::ostream &os, const Trace &trace) {
  Tracklimits tracklimits = trace.getTracklimits();
  for (const Node &n : tracklimits.first) {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 0 << ' ' << n.id << std::endl;
  }
  for (const Node &n : tracklimits.second) {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 1 << ' ' << n.id << std::endl;
  }
  return os;
}

std::vector<Point> Trace::getPath() const {
  std::vector<Point> res;
  res.reserve(this->size());

  std::shared_ptr<Connection> aux = this->p;
  while (aux) {
    res.push_back(aux->edge.midPointGlobal());
    aux = aux->before;
  }
  std::reverse(res.begin(), res.end());
  return res;
}

Tracklimits Trace::getTracklimits() const {
  Tracklimits res;
  if (this->empty()) return res;

  std::vector<const Edge*> edges;
  edges.reserve(this->size());
  std::shared_ptr<Connection> aux = this->p;
  while (aux) {
    edges.push_back(&(aux->edge));
    aux = aux->before;
  }

  res.first.reserve(this->size());
  res.second.reserve(this->size());
  Point pAnt(-50, edges.back()->midPointGlobal().y);  // This only works in a global, the -50 is arbitrary

  const Node *left, *firstLeft;
  const Node *right, *firstRight;

  size_t edgeInd = 0;
  for (auto it = edges.crbegin(); it != edges.crend(); it++) {
    const Edge &e = *(*it);
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
         (!this->isLoopClosed() or edgeInd < edges.size() - 3 or !Vector::pointBehind(res.first.front().pointGlobal(), left->pointGlobal(), pAntPAct)))) {
      if (*left == *firstLeft) {
        res.first.push_back(*firstLeft);
      }
      else res.first.push_back(*left);
    }
    if (res.second.empty() or
        (*right != res.second.back() and
         !Vector::pointBehind(right->pointGlobal(), res.second.back().pointGlobal(), pAntPAct) and
         (!this->isLoopClosed() or edgeInd < edges.size() - 3 or !Vector::pointBehind(res.second.front().pointGlobal(), right->pointGlobal(), pAntPAct)))) {
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