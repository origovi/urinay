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

Trace::Connection::Connection(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed, std::shared_ptr<Connection> before)
    : edgeInd(edgeInd), before(before), heur(heur), size(before ? (before->size + 1) : 1), avgEdgeLen(before ? before->avgEdgeLen + ((edgeLen - before->avgEdgeLen) / before->size + 1) : edgeLen), loopClosed(before ? (before->loopClosed or loopClosed) : loopClosed) {}

bool Trace::Connection::containsEdge(const size_t &_edgeInd) const {
  return edgeInd == _edgeInd || (before ? before->containsEdge(_edgeInd) : false);
}

Trace::Trace(std::shared_ptr<Connection> p)
    : p(p) {}

/* ----------------------------- Public Methods ----------------------------- */

Trace::Trace()
    : p(nullptr) {}

Trace::Trace(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed) {
  this->p = std::make_shared<Connection>(edgeInd, heur, edgeLen, loopClosed, nullptr);
}

void Trace::addEdge(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed) {
  this->p = std::make_shared<Connection>(edgeInd, heur, edgeLen, loopClosed, this->p);
}

bool Trace::empty() const {
  return not this->p;
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

const size_t &Trace::edgeInd() const {
  ROS_ASSERT(not empty());
  return this->p->edgeInd;
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
    return 0.0;
  else
    return heur() + before().sumHeur();
}

bool Trace::containsEdge(const size_t &edgeInd) const {
  if (empty())
    return false;
  else
    return this->p->containsEdge(edgeInd);
}

void Trace::clear() {
  this->p = nullptr;
}

// Useful to print a Trace
std::ostream &operator<<(std::ostream &os, const Trace &trace) {
  os << "T(";
  if (!trace.empty()) os << *(trace.p);
  return os << ")";
}