#include "structures/Trace.hpp"

/* ----------------------------- Private Methods ---------------------------- */

Trace::Connection::Connection(const size_t &edgeInd, const float &heur, std::shared_ptr<Connection> before)
    : edgeInd(edgeInd), before(before), heur(heur), size(before ? (before->size + 1) : 1) {}

bool Trace::Connection::containsEdge(const size_t &_edgeInd) const {
  return edgeInd == _edgeInd || (before ? before->containsEdge(_edgeInd) : false);
}

Trace::Trace(std::shared_ptr<Connection> p)
    : p(p) {}

/* ----------------------------- Public Methods ----------------------------- */

Trace::Trace()
    : p(nullptr) {}

Trace::Trace(const size_t &edgeInd, const float &heur) {
  this->p = std::make_shared<Connection>(edgeInd, heur, nullptr);
}

void Trace::addEdge(const size_t &edgeInd, const float &heur) {
  this->p = std::make_shared<Connection>(edgeInd, heur, this->p);
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

// CANT RUN IF NOT SURE THE CONE IS IN THIS
Trace Trace::getTraceByEdgeInd(const size_t &index) {
  if (edgeInd() == index)
    return *this;
  else
    return Trace(this->p->before).getTraceByEdgeInd(index);
}

Trace Trace::getTraceBySize(const size_t &size) const {
  if (this->size() <= size)
    return *this;
  else
    return Trace(this->p->before).getTraceBySize(size);
}

const size_t &Trace::edgeInd() const {
  ROS_ASSERT(not empty());
  return this->p->edgeInd;
}

const float &Trace::heur() const {
  ROS_ASSERT(not empty());
  return this->p->heur;
}

float Trace::sumHeur() const {
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