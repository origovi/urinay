/**
 * @file TreeBuffer.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the TreeBuffer class specification
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

/**
 * @brief Represents a buffer of a Trace tree, i.e. a set of traces all
 * emerging from the same midpoint.
 * It exists to optimize tree search not having to create the complete tree
 * at every iteration.
 */
class TreeBuffer {
 private:
  std::list<Trace> tree_;

 public:
  TreeBuffer();

  /**
   * @brief Inserts a new Trace into the buffer.
   *
   * @param[in] trace
   */
  void insert(const Trace &trace);

  /**
   * @brief Updates the tree so it removes all Trace(s) having no Edge index
   * \a lastEdgeInd and detaches all remaining Trace(s) from Edge index
   * \a lastEdgeInd.
   *
   * @param[in] lastEdgeInd
   */
  void update(const size_t &lastEdgeInd);

  /**
   * @brief Fills the queue \a queue with the contents of tree_.
  */
  void fillQueue(std::queue<Trace> &queue) const;

  /**
   * @brief Checks whether or not there is not valid data.
  */
  bool empty() const;

  /**
   * @brief Clears all content of the buffer.
   */
  void clear();
};