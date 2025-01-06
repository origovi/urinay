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
  : params_(params) {
    this->currentStep_ = 0;
}

// void TraceBuffer::update(const size_t &lastEdgeInd) {
//   auto it = this->buffer_.begin();

//   while (it != this->buffer_.end()) {
//     if (not it->containsEdge(lastEdgeInd) or it->size() == 1) {
//       it = this->buffer_.erase(it);
//     }
//     else {
//       it++;
//     }
//   }
  
//   // WARNING: Assuming all traces in this->buffer_ come from SAME real tree.
//   // (not having inserted traces with different origin)
//   for (Trace &t : this->buffer_) {
//     t.detachFrom(lastEdgeInd);
//   }
// }

void TraceBuffer::newStep() const {
  this->currentStep_++;
}

void TraceBuffer::prune() {

  Trace best = this->bestTrace();

  // 1. Remove shorter traces
  auto it = this->begin();
  while (it != this->end()) {
    if (it->size() < best.size() or it->sumHeur()*params_.prune_same_height_heur_factor > best.sumHeur()) {
      it = this->erase(it);
    } else {
      it++;
    }
  }

  // //DEBUG
  // std::cout << "FROM: ";
  // for (const Trace &t : *this)  {
  //   std::cout << t.size() << ' ' << t.sumHeur() << std::endl;
  // }

  // // 2. Keep k best traces
  // 2.1. We construct a max heap to keep the k best
  std::vector<Trace> max_heap;
  max_heap.reserve(this->params_.best_search_options_to_keep);
  for (const Trace &t : *this) {
    if (max_heap.size() < this->params_.best_search_options_to_keep) {
      max_heap.push_back(t);
      std::push_heap(max_heap.begin(), max_heap.end());  // Maintain max-heap property
    } else if (t < max_heap.front()) {
      std::pop_heap(max_heap.begin(), max_heap.end());   // Remove the largest in the heap
      max_heap.back() = t;  // Replace it with the new smaller number
      std::push_heap(max_heap.begin(), max_heap.end());  // Restore heap property
    }
  }
  std::sort(max_heap.begin(), max_heap.end());

  // 2.2. Convert heap back to std::list
  this->clear();
  for (const Trace &t : max_heap) {
    this->push_back(t);
  }

  // std::cout << "TO: ";
  // for (const Trace &t : *this)  {
  //   std::cout << t.size() << ' ' << t.sumHeur() << std::endl;
  // }

}

Trace TraceBuffer::bestTrace() const {
  Trace best;
  for (const Trace &t : *this) {
    if (t < best) best = t;
  }
  return best;
}

size_t TraceBuffer::height() const {
  return bestTrace().size();
}