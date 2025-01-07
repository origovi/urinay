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
#include <queue>

#include "structures/Trace.hpp"
#include "utils/Params.hpp"

/**
 * @brief Represents a buffer of a search Trace tree, i.e. a set of traces
 * each representing a possible path for the car to take.
 * It provides methods to prune the possibilities and other useful
 * functionality.
 */
class TraceBuffer : public std::list<Trace> {
 private:
  /**
   * @brief Search parameters.
   */
  const Params::WayComputer::Search params_;

 public:
  TraceBuffer(const Params::WayComputer::Search &params);

  /**
   * @brief Prunes the buffer by removing full Trace(s) that are far from
   * the best.
   */
  void prune();

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