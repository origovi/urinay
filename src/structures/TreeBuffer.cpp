/**
 * @file TreeBuffer.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the TreeBuffer class member functions implementation
 * @version 1.0
 * @date 2024-12-15
 *
 * @copyright Copyright (c) 2024 TUfast e.V.
 */

#include "structures/TreeBuffer.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

TreeBuffer::TreeBuffer() {}

void TreeBuffer::insert(const Trace &trace) {
  this->tree_.push_back(trace);
}

void TreeBuffer::update(const size_t &lastEdgeInd) {
  auto it = this->tree_.begin();

  while (it != this->tree_.end()) {
    if (not it->containsEdge(lastEdgeInd) or it->size() == 1) {
      it = this->tree_.erase(it);
    }
    else {
      it++;
    }
  }
  
  // WARNING: Assuming all traces in this->tree_ come from SAME real tree.
  // (not having inserted traces with different origin)
  for (Trace &t : this->tree_) {
    t.detachFrom(lastEdgeInd);
  }
}

void TreeBuffer::fillQueue(std::queue<Trace> &queue) const {
  for (const Trace &t : this->tree_) {
    queue.push(t);
  }
}

bool TreeBuffer::empty() const {
  return this->tree_.empty();
}

void TreeBuffer::clear() {
  this->tree_.clear();
}