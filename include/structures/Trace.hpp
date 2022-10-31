/**
 * @file Trace.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Trace class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ros/ros.h>

#include <iostream>
#include <memory>

#include "structures/Edge.hpp"

/**
 * @brief Represents a trace, i.e. an edge path in the tree search.
 * It is implemented using shared_ptr, this way we can clone an object and
 * append an Edge with O(1) time.
 */
class Trace {
 private:
  /**
   * @brief Inner class of Trace, contains information about last append
   * and a pointer to the Connection before.
   */
  struct Connection {
    /**
     * @brief A pointer (index) to the Edge it represents.
     */
    const size_t edgeInd;

    /**
     * @brief A shared pointer to the Connection that goes before this.
     */
    const std::shared_ptr<Connection> before;

    /**
     * @brief Size of the Trace from this Connection
     */
    const size_t size;

    /**
     * @brief The heuristic of this append.
     */
    const double heur;

    /**
     * @brief The average Edge length of all the Trace.
     */
    const double avgEdgeLen;

    /**
     * @brief Whether or not the Trace has closed the loop
     * (and thus it contains (or may contain) repeated Edge(s)).
     */
    const bool loopClosed;

    /**
     * @brief Construct a new Connection object.
     * 
     * @param[in] edgeInd 
     * @param[in] heur 
     * @param[in] edgeLen 
     * @param[in] loopClosed 
     * @param[in] before 
     */
    Connection(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed, std::shared_ptr<Connection> before);

    /**
     * @brief Returns if there is \a _edgeInd in the Connection chain.
     * 
     * @param[in] _edgeInd 
     */
    bool containsEdge(const size_t &_edgeInd) const;

    /**
     * @brief Cout operator.
     * 
     * @param[in,out] os 
     * @param[in] conn 
     */
    friend std::ostream &operator<<(std::ostream &os, const Connection &conn) {
      if (conn.before != nullptr) {
        os << *(conn.before);
      }
      return os << conn.edgeInd << " -> ";
    }
  };

  /**
   * @brief ONLY ATTRIBUTE, a shared pointer to a Connection.
   */
  std::shared_ptr<Connection> p;

  /**
   * @brief Construct a new Trace object having \a p as a pointer to the
   * Connection chain.
   * 
   * @param[in] p 
   */
  Trace(std::shared_ptr<Connection> p);

 public:
  /**
   * @brief Construct a new Trace object.
   */
  Trace();

  /**
   * @brief Construct a new Trace object and the first Connection object.
   * 
   * @param[in] edgeInd 
   * @param[in] heur 
   * @param[in] edgeLen 
   * @param[in] loopClosed 
   */
  Trace(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed = false);

  /**
   * @brief Appends an Edge as a Connection with the following data.
   * 
   * @param[in] edgeInd 
   * @param[in] heur 
   * @param[in] edgeLen 
   * @param[in] loopClosed 
   */
  void addEdge(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed = false);

  /**
   * @brief Consults if the Trace is empty.
   */
  bool empty() const;

  /**
   * @brief Returns the size of the Trace, i.e. how long is the Connection chain.
   */
  size_t size() const;

  /**
   * @brief Returns a Trace containing the Connection chain of the Connection
   * before.
   */
  Trace before() const;

  /**
   * @brief Returns a Trace containing the first (initial) Connection.
   */
  Trace first() const;

  /**
   * @brief Returns the last Edge index.
   */
  const size_t &edgeInd() const;

  /**
   * @brief Returns the last Connection heuristic.
   */
  const double &heur() const;

  /**
   * @brief Returns the last Connection's average Edge length.
   */
  double avgEdgeLen() const;

  /**
   * @brief Returns whether of not the loop is closed from last
   * Connection or before.
   */
  bool isLoopClosed() const;

  /**
   * @brief Returns the sum of heuristics of the Connection chain.
   */
  double sumHeur() const;

  /**
   * @brief Checks if there is any Connection with Edge index \a edgeInd.
   * 
   * @param[in] edgeInd 
   */
  bool containsEdge(const size_t &edgeInd) const;

  /**
   * @brief Clears the Connection chain.
   */
  void clear();

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] trace 
   */
  friend std::ostream &operator<<(std::ostream &os, const Trace &trace);
};