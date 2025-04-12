/**
 * @file TraceBuffer.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the TraceBuffer class member functions implementation
 * @version 1.0
 * @date 2024-12-15
 *
 * @copyright Copyright (c) 2024 TUfast e.V.
 */

#include "structures/TraceBuffer.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

TraceBuffer::TraceBuffer(const Params::WayComputer::Search &params)
  : params_(params) {}

void TraceBuffer::prune() {

  Trace best = this->bestTrace();

  // Remove shortest (num midpoints) traces
  auto it = this->begin();
  while (it != this->end()) {
    if (it->trace.size() < best.size()) {
      it = this->erase(it);
    } else {
      it++;
    }
  }
}

void TraceBuffer::setEdgeAsDefinitive(const Edge &nextEdge) {
  // Internally, what this is doing, is just remove all Trace(s) that do not
  // have nextEdge as their first edge of the tree search (buffer) and pops
  // the tree search buffer queue so the next first Edge is the second.
  auto it = this->begin();
  while (it != this->end()) {
    if (*(it->buffer.front()) != nextEdge) {
      it = this->erase(it);
    } else {
      it->buffer.pop_front();
      it++;
    }
  }
}

TraceWithBuffer TraceBuffer::bestTraceWithBuffer() const {
  TraceWithBuffer best;
  for (const TraceWithBuffer &t : *this) {
    if (t.trace < best.trace) best = t;
  }
  return best;
}

Trace TraceBuffer::bestTrace() const {
  return this->bestTraceWithBuffer().trace;
}

size_t TraceBuffer::height() const {
  return bestTrace().size();
}