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

Trace::Connection::Connection(const Edge* const edge, const double &heur, const bool &loopClosed, std::shared_ptr<Connection> before)
    : edge(edge), before(before), heur(heur),
      sumHeur(before ? before->sumHeur + heur : heur), size(before ? (before->size + 1) : 1),
      avgEdgeLen(before ? before->avgEdgeLen + ((edge->len - before->avgEdgeLen) / (before->size + 1)) : edge->len),
      loopClosed(before ? (before->loopClosed or loopClosed) : loopClosed),
      connectionsSinceLoopClosed(before && before->connectionsSinceLoopClosed >= 1 ? before->connectionsSinceLoopClosed + 1 : (loopClosed ? 1 : 0)) {}

bool Trace::Connection::containsEdge(const Edge &_edge) const {
  return *edge == _edge || (before ? before->containsEdge(_edge) : false);
}

Trace::Trace(std::shared_ptr<Connection> p)
    : p(p) {}

/* ----------------------------- Public Methods ----------------------------- */

Trace::Trace()
    : p(nullptr) {}

Trace::Trace(const Edge &edge, const double &heur, const bool &loopClosed, const Trace &previous) {
  this->p = std::make_shared<Connection>(&edge, heur, loopClosed, previous.empty() ? nullptr : previous.p);
}

void Trace::addEdge(const Edge &edge, const double &heur, const bool &loopClosed) {
  this->p = std::make_shared<Connection>(&edge, heur, loopClosed, this->p);
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

Trace Trace::before() const {
  ROS_ASSERT(not empty());
  return Trace(p->before);
}

Trace Trace::first() const {
  ROS_ASSERT(not empty());
  std::shared_ptr<Connection> lastNotEmpty = p;
  while (lastNotEmpty->before != nullptr) {
    lastNotEmpty = lastNotEmpty->before;
  }
  return Trace(lastNotEmpty);
}

const Edge &Trace::edge() const {
  ROS_ASSERT(not empty());
  return *(this->p->edge);
}

const double &Trace::heur() const {
  ROS_ASSERT(not empty());
  return this->p->heur;
}

double Trace::avgEdgeLen() const {
  if (empty()) return 0.0;
  else return this->p->avgEdgeLen;
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

uint32_t Trace::connectionsSinceLoopClosed() const {
  if (empty())
    return 0;
  else
    return this->p->connectionsSinceLoopClosed;
}

Trace Trace::trimLoopClosure() const {
  Trace res = *this;
  while (!res.empty() and res.connectionsSinceLoopClosed() >= 2) {
    res = res.before();
  }
  return res;
}

void Trace::clear() {
  this->p = nullptr;
}

bool Trace::operator<(const Trace &t) const {
  return this->size() > t.size() or (this->size() == t.size() and this->sumHeur() < t.sumHeur());
}

// Useful to print a Trace
std::ostream &operator<<(std::ostream &os, const Trace &trace) {
  os << "T(";
  if (!trace.empty()) os << *(trace.p);
  return os << ")";
}