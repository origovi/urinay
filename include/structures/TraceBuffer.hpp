/**
 * @file TraceBuffer.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the TraceBuffer class specification
 * @version 1.0
 * @date 2024-12-15
 *
 * @copyright Copyright (c) 2024 TUfast e.V.
 */

#pragma once

#include <ros/ros.h>
#include <list>
#include <deque>

#include "structures/Trace.hpp"
#include "utils/Params.hpp"

/**
 * @brief Auxiliar type to help with the search.
 * Represents a trace and a buffer containing the elements in the current
 * tree search.
 */
class TraceWithBuffer {
 public:
  Trace trace;
  std::deque<const Edge*> buffer;
  TraceWithBuffer(const Trace &t, const Edge *firstEdge = nullptr)
    : trace(t) {
    if (firstEdge) buffer.push_back(firstEdge);
  }
  TraceWithBuffer() {}
};

/**
 * @brief Represents a buffer of a search Trace tree, i.e. a set of traces
 * each representing a possible path for the car to take.
 * It provides methods to prune the possibilities and other useful
 * functionality.
 */
class TraceBuffer : public std::list<TraceWithBuffer> {
 private:
  /**
   * @brief Search parameters.
   */
  const Params::WayComputer::Search params_;

 public:

  TraceBuffer(const Params::WayComputer::Search &params);

  /**
   * @brief Prunes the buffer by removing full Trace(s) that are shorter than
   * the best.
   */
  void prune();

  /**
   * @brief Adds the next edge to the definitive path.
   * 
   * @param[in] nextEdge represents the next valid Edge that should be part
   * of the definitive path
   */
  void setEdgeAsDefinitive(const Edge &nextEdge);

  /**
   * @brief Returns the best TraceWithBuffer (according to Trace::operator<).
   * WARNING, if empty, returns an empty Trace!
   */
  TraceWithBuffer bestTraceWithBuffer() const;

  /**
   * @brief Returns the best Trace (according to Trace::operator<).
   * WARNING, if empty, returns an empty Trace!
   */
  Trace bestTrace() const;

  /**
   * @brief Returns size of the best Trace in the buffer.
   * WARNING, implementation correctness depends on bestTrace impl.
   */
  size_t height() const;
};